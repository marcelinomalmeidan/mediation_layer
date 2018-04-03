
#include "mediation_layer/threads.h"

namespace threads {

void MediationLayerThread(const double &rate) {
    ROS_DEBUG("[mediation layer] Mediation Layer Thread started!");

    // Rate at which this thread will run
    // double rate = 300.0;  // Rate in Hz
    double dt = 1/rate;
    ros::Rate loop_rate(rate);
    const double quad_radius = 0.25;

    // Run the Mediation Layer loop
    while (ros::ok()) {
        std::vector<Eigen::Vector3d> quad_positions;

        // Update mediation layer
        pthread_mutex_lock(&mutexes_.m_ml_class);

            // Reset reaction forces
            globals_.obj_mid_layer.ResetForces();

            // Update reaction forces between vehicles
            globals_.obj_mid_layer.UpdateVehicleReactionForces();

            // Update reaction forces between the quads and the walls
            globals_.obj_mid_layer.UpdateArenaReactionForces();

            // Integrate ML diff. equations and update references
            globals_.obj_mid_layer.UpdateMediationLayerOutputs(dt);

            // Publish new references into ROS
            globals_.obj_mid_layer.PublishMLReferences();

            // Get all quad positions
            globals_.obj_mid_layer.GetQuadPositions(&quad_positions);

        pthread_mutex_unlock(&mutexes_.m_ml_class);

        // Check whether balloons have popped
        pthread_mutex_lock(&mutexes_.m_balloons);
            for (uint i = 0; i < globals_.balloons.size(); i++) {
                for (uint j = 0; j < quad_positions.size(); j++) {
                    // We don't check if balloon is already popped
                    if (globals_.balloons[i].is_popped()) {
                        continue;
                    }

                    // If balloon isn't popped, we check proximity with quads
                    const double dist = 
                        (globals_.balloons[i].position_ - quad_positions[j]).norm();
                    if (dist < quad_radius) {
                        globals_.balloons[i].popped_ = true;
                        ROS_INFO("[mediation layer]: %s balloon popped!", 
                                 globals_.balloons[i].name_.c_str());
                    }
                }
            }
        pthread_mutex_unlock(&mutexes_.m_balloons);

        loop_rate.sleep();
    }
    ROS_DEBUG("Exiting Mediation Layer Thread...");
}

void HeartbeatThread() {
    ROS_DEBUG("[mediation layer] Heartbeat Thread started!");

    const double rate = 5.0;  // Rate in Hz
    ros::Rate loop_rate(rate);

    // Timeout for heartbeats
    const double timeout = 2.0;

    while (ros::ok()) {
        pthread_mutex_lock(&mutexes_.m_ml_class);

        ros::Time time_now = ros::Time::now();
        
        std::set<QuadData>::iterator it;
        for(it = globals_.obj_mid_layer.quads_.begin(); 
            it != globals_.obj_mid_layer.quads_.end(); ++it) {

            ros::Time last_ref = it->last_reference_stamp;
            if (it->ref_is_active && (time_now - last_ref).toSec() > timeout) {
                it->ref_is_active = false;
                ROS_WARN("[mediation layer] Quad %s references inactive!", it->name.c_str());
            }

            ros::Time last_meas = it->last_measurement_stamp;
            if (it->odom_is_active && (time_now - last_meas).toSec() > timeout) {
                it->odom_is_active = false;
                ROS_WARN("[mediation layer] Quad %s odometry inactive!", it->name.c_str());
            }
        }

        pthread_mutex_unlock(&mutexes_.m_ml_class);

        loop_rate.sleep();
    }
}

void GameStatePubThread(const double &rate) {
    ros::Rate loop_rate(rate);
    std::set<QuadData> quad_list;
    ros::Publisher pub_game_state;


    while (ros::ok()) {
        mg_msgs::GameState game_state;

        pthread_mutex_lock(&mutexes_.m_ml_class);
            quad_list = globals_.obj_mid_layer.quads_;
            pub_game_state = globals_.obj_mid_layer.pub_game_state_;
        pthread_mutex_unlock(&mutexes_.m_ml_class);

        std::set<QuadData>::iterator it;
        for(it = quad_list.begin(); it != quad_list.end(); ++it) {

            if(it->odom_is_active) {
                game_state.GameState.push_back(it->vehicle_odom);
            }
        }

        pub_game_state.publish(game_state);

        loop_rate.sleep();
    }
}

void tfThread(const double &rate) {
    static ros::Rate loop_rate(rate);
    static std::set<QuadData> quad_list;
    static tf::TransformBroadcaster br;
    static std::string world_frame = "world";
    std::string vehicle_frame, camera_frame;
    tf::Vector3 camPos(0.1, 0.0, 0.0);

    // Get camera rotation
    Eigen::Quaterniond q_cam1(cos(M_PI/4.0), 0.0, sin(M_PI/4.0), 0.0);
    Eigen::Quaterniond q_cam2(cos(M_PI/4.0), 0.0, 0.0, -sin(M_PI/4.0));
    Eigen::Quaterniond q_cam = q_cam1*q_cam2;

    while (ros::ok()) {

        pthread_mutex_lock(&mutexes_.m_ml_class);
            quad_list = globals_.obj_mid_layer.quads_;
        pthread_mutex_unlock(&mutexes_.m_ml_class);

        std::set<QuadData>::iterator it;
        for(it = quad_list.begin(); it != quad_list.end(); ++it) {
            if(!it->odom_is_active) {
                continue;
            }
            
            tf::Transform transform, transform_cam;
            geometry_msgs::Pose pose = it->vehicle_odom.pose.pose;

            // Set vehicle pose
            transform.setOrigin( tf::Vector3(pose.position.x,
                                           pose.position.y, 
                                           pose.position.z));
            tf::Quaternion q(pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w);
            transform.setRotation(q);

            // Set camera pose
            transform_cam.setOrigin(camPos);
            tf::Quaternion tf_qcam(q_cam.x(), q_cam.y(), q_cam.z(), q_cam.w());
            transform_cam.setRotation(tf_qcam);

            // Get time
            ros::Time time_now = ros::Time::now();
            
            // Frame ids
            vehicle_frame = it->name;
            camera_frame = it->name + "/camera";

            // Send transforms
            br.sendTransform(tf::StampedTransform(transform, time_now,
                                                  world_frame, vehicle_frame));
            br.sendTransform(tf::StampedTransform(transform_cam, time_now,
                                                  vehicle_frame, camera_frame));
        }
    
        loop_rate.sleep();
    }
}

void StaticObjectsVisualizationThread() {
	ROS_DEBUG("[mediation layer] Static Objects Visualization Thread started!");

	const std::string frame_id = "world";
	const std::string ground_ns = "ground";

	const std_msgs::ColorRGBA ground_color = visualization_functions::Color::DarkGreen();
	const double gound_transparency = 1.0;
    const std::string balloon_mesh = "balloon.stl";
    const std::string balloon_ns = "balloons";
    const double balloon_size = 20.0;
    const double balloon_transparency = 1.0;
    const Eigen::Vector3d balloon_offset(0.0, 0.0, -0.75);
    const Eigen::Quaterniond balloon_att(1.0, 0.0, 0.0, 0.0);
    BoxPlanes arena_box;
    std::vector<Balloon> balloons;
    ros::Publisher pub_vis;

	// Visualization markers for the arena
    visualization_msgs::MarkerArray static_obj_markers;

    const double rate = 2.0;  // Rate in Hz
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
    	static_obj_markers.markers.clear();

	    pthread_mutex_lock(&mutexes_.m_ml_class);
            pub_vis = globals_.obj_mid_layer.pub_vis_;
            arena_box = globals_.obj_mid_layer.arena_box_;
	    pthread_mutex_unlock(&mutexes_.m_ml_class);
        pthread_mutex_lock(&mutexes_.m_balloons);
            balloons = globals_.balloons;
        pthread_mutex_unlock(&mutexes_.m_balloons);

	    // Get corners from arena
	    Eigen::Vector3d corner1 = arena_box.DLB_;
	    Eigen::Vector3d corner2 = arena_box.DRF_;
	    Eigen::Vector3d corner3(corner1[0], corner2[1], corner1[2]);
	    Eigen::Vector3d corner4(corner2[0], corner1[1], corner1[2]);

		visualization_functions::PlaneMarker(corner1, corner2,
                 corner3, corner4, frame_id, ground_ns, ground_color,
                 gound_transparency, &static_obj_markers);

	    arena_box.VisualizeBox(frame_id, &static_obj_markers);

        // Get balloon markers
        for (uint i = 0; i < balloons.size(); i++) {
            if (balloons[i].is_popped()) {  // Transparent balloon marker
                const double transparent = 0.0;
                visualization_functions::MeshMarker(balloons[i].position_+balloon_offset,
                    balloon_att, frame_id, balloon_ns, balloon_mesh, balloon_size,
                    balloons[i].color_, transparent, i, &static_obj_markers);
            } else {
                visualization_functions::MeshMarker(balloons[i].position_+balloon_offset,
                    balloon_att, frame_id, balloon_ns, balloon_mesh, balloon_size,
                    balloons[i].color_, balloon_transparency, i, &static_obj_markers);
            }
        }


	    pub_vis.publish(static_obj_markers);

	    loop_rate.sleep();

    }
}

void VisualizationThread(const double &rate) {
	ROS_DEBUG("[mediation layer] Visualization Thread started!");

    // Rate at which this thread will run
    // const double rate = 30.0;  // Rate in Hz
    const double dt = 1/rate;
    const double gravity = 9.81;

    // Visualization marker parameters
    const std::string frame_id = "world";
    const std::string quad_mesh = "quadrotor_base.dae";
    const double quad_size = 0.375;
    const double sphere_transparency = 0.1;
    const double shield_transparency = 0.25;
    const std_msgs::ColorRGBA frame_color = visualization_functions::Color::White();
    const std_msgs::ColorRGBA sphere_color = visualization_functions::Color::White();
    const std_msgs::ColorRGBA text_color = visualization_functions::Color::Black();
    const std_msgs::ColorRGBA force_color = visualization_functions::Color::Red();
    const std_msgs::ColorRGBA shield_color = visualization_functions::Color::Teal();
    const double reference_transparency = 0.5;
    const double pos_transparency = 1.0;
    const double reference_size = 0.1;
    visualization_msgs::MarkerArray quadArray;
    std::set<QuadData> quad_list;
    ros::Publisher pub_vis;
    double max_acc;

    pthread_mutex_lock(&mutexes_.m_ml_class);
    	const double sphere_size = globals_.obj_mid_layer.d_thresh_;
    pthread_mutex_unlock(&mutexes_.m_ml_class);

    ros::Rate loop_rate(rate);

    while (ros::ok()) {
    	// Get time for when this task started
        const ros::Time t0 = ros::Time::now();

        // Empty visualization markers
        quadArray.markers.clear();

        pthread_mutex_lock(&mutexes_.m_ml_class);
            quad_list = globals_.obj_mid_layer.quads_;
            pub_vis = globals_.obj_mid_layer.pub_vis_;
            max_acc = globals_.obj_mid_layer.max_acc_;
        pthread_mutex_unlock(&mutexes_.m_ml_class);

        std::set<QuadData>::iterator it;
        for(it = quad_list.begin(); it != quad_list.end(); ++it) {

        	if(it->ref_is_active) {
        		// Get some parameters for visualization
                const Eigen::Vector3d Acc2 = helper::Vec32vec3d(it->ml_reference.Acc) + 
                                             Eigen::Vector3d(0.0, 0.0, gravity);
                const Eigen::Matrix3d orientationFrame = helper::Triad(Acc2, it->ml_reference.yaw);
                const Eigen::Quaterniond orientationMesh = helper::TriadQuat(Acc2, it->ml_reference.yaw + M_PI/4.0);
                const Eigen::Vector3d ml_ref_pos = helper::Point2vec3d(it->ml_reference.Pos);

                // Get marker for reference position of each quad
                Eigen::Vector3d reference = helper::Point2vec3d(it->reference.Pos);
                visualization_functions::SphereMarker(reference,
                            frame_id, it->name, reference_size, it->color, 
                            reference_transparency, 0, &quadArray);

                // Get quad mesh
                visualization_functions::MeshMarker(ml_ref_pos, orientationMesh,
                            frame_id, it->name, quad_mesh, quad_size, it->color, 
                            reference_transparency, 1, &quadArray);

                visualization_functions::NameMarker(ml_ref_pos, it->name,
                            frame_id, it->name, text_color, 2, &quadArray);

        	}

            // Some of the markers below overwrites the ones above
            if(it->odom_is_active) {
                // Get some parameters for visualization
                const geometry_msgs::Quaternion q = it->vehicle_odom.pose.pose.orientation;
                Eigen::Quaterniond q_mesh(q.w, q.x, q.y, q.z);
                const Eigen::Matrix3d orientation_frame = q_mesh.normalized().toRotationMatrix();
                const Eigen::Quaterniond rot_quad(cos(M_PI/8.0), 0.0, 0.0, sin(M_PI/8.0)); // Quad mesh is rotated by 45deg
                q_mesh = q_mesh*rot_quad;
                Eigen::Vector3d position = 
                    helper::Point2vec3d(it->vehicle_odom.pose.pose.position);
                
                // Get marker for sphere of influence of each quad
                visualization_functions::SphereMarker(position,
                            frame_id, it->name, sphere_size, sphere_color, 
                            sphere_transparency, 3, &quadArray);

                // Get shield plane marker
                if(it->has_shield) {
                    double len = 0.5*sphere_size, wid = sphere_size, height = sphere_size;
                    Eigen::Vector3d x_dir(cos(it->ml_reference.yaw), sin(it->ml_reference.yaw), 0.0);
                    visualization_functions::CuboidMarker(position + 0.5*len*x_dir,
                        frame_id, it->name, len, wid, height, it->ml_reference.yaw,
                        shield_color, shield_transparency, 4, &quadArray);
                    // visualization_functions::PlaneMarker(position,
                    //         frame_id, it->name, sphere_size, sphere_color,
                    //         shield_transparency, 4, it->ml_reference.yaw, &quadArray);
                }

                // Get quad mesh
                visualization_functions::MeshMarker(position, q_mesh,
                            frame_id, it->name, quad_mesh, quad_size, it->color, 
                            pos_transparency, 5, &quadArray);

                // Get force arrow around quad
                visualization_functions::ForceMarker(position, 
                            it->force_field, max_acc, frame_id, 
                            it->name, force_color, 6, &quadArray);

                visualization_functions::NameMarker(position, it->name,
                            frame_id, it->name, text_color, 2, &quadArray);

                // Get triad frame arrows
                const double arrowLength = 0.2; 
                visualization_functions::FrameMarker(position, orientation_frame,
                            frame_id, it->name, frame_color, 7, arrowLength, &quadArray);
            }


    	}

    	if(quadArray.markers.size() > 0) {
    		pub_vis.publish(quadArray);
    	}

		loop_rate.sleep();
    }


    ROS_DEBUG("Exiting Visualization Thread...");
}

}  // namespace threads