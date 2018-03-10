
#include "mediation_layer/threads.h"

namespace threads {

void MediationLayerThread(const double &rate) {
    ROS_DEBUG("[mediation layer] Mediation Layer Thread started!");

    // Rate at which this thread will run
    // double rate = 300.0;  // Rate in Hz
    double dt = 1/rate;
    ros::Rate loop_rate(rate);


    // Run the Mediation Layer loop
    std::set<QuadData>::iterator it1, it2;
    while (ros::ok()) {
        // Get time for when this task started
        const ros::Time t0 = ros::Time::now();

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

        pthread_mutex_unlock(&mutexes_.m_ml_class);

        // ros::Duration ml_time = ros::Time::now() - t0;
        // ROS_INFO("Mediation Layer execution time: %f", ml_time.toSec());

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

void StaticObjectsVisualizationThread() {
	ROS_DEBUG("[mediation layer] Static Objects Visualization Thread started!");

	const std::string frame_id = "world";
	const std::string ground_ns = "ground";

	const std_msgs::ColorRGBA ground_color = visualization_functions::Color::DarkGreen();
	const double gound_transparency = 1.0;

	// Visualization markers for the arena
    visualization_msgs::MarkerArray static_obj_markers;

    const double rate = 1.0;  // Rate in Hz
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
    	static_obj_markers.markers.clear();

	    pthread_mutex_lock(&mutexes_.m_ml_class);
	    	MediationLayer local_obj_mid_layer = globals_.obj_mid_layer;
	    pthread_mutex_unlock(&mutexes_.m_ml_class);

	    // Get corners from arena
	    Eigen::Vector3d corner1 = local_obj_mid_layer.arena_box_.DLB_;
	    Eigen::Vector3d corner2 = local_obj_mid_layer.arena_box_.DRF_;
	    Eigen::Vector3d corner3(corner1[0], corner2[1], corner1[2]);
	    Eigen::Vector3d corner4(corner2[0], corner1[1], corner1[2]);

		visualization_functions::PlaneMarker(corner1, corner2,
                 corner3, corner4, frame_id, ground_ns, ground_color,
                 gound_transparency, &static_obj_markers);

	    local_obj_mid_layer.arena_box_.VisualizeBox(frame_id, &static_obj_markers);

	    local_obj_mid_layer.pub_vis_.publish(static_obj_markers);

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
    const double quad_size = 0.375;
    const double sphere_transparency = 0.05;
    const std_msgs::ColorRGBA frame_color = visualization_functions::Color::White();
    const std_msgs::ColorRGBA sphere_color = visualization_functions::Color::White();
    const std_msgs::ColorRGBA text_color = visualization_functions::Color::Black();
    const std_msgs::ColorRGBA force_color = visualization_functions::Color::Red();
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
        	// local_obj_mid_layer = globals_.obj_mid_layer;
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
                            reference_transparency, 1, &quadArray);

                // Get quad mesh
                visualization_functions::MeshMarker(ml_ref_pos, orientationMesh,
                            frame_id, it->name, quad_size, it->color, 
                            reference_transparency, 2, &quadArray);

                visualization_functions::NameMarker(ml_ref_pos, it->name,
                            frame_id, it->name, text_color, 4, &quadArray);

        	}

            // Some of the markers below overwrites the ones above
            if(it->odom_is_active) {
                // Get some parameters for visualization
                const geometry_msgs::Quaternion q = it->vehicle_odom.pose.pose.orientation;
                Eigen::Quaterniond q_mesh(q.w, q.x, q.y, q.z);
                const Eigen::Matrix3d orientation_frame = q_mesh.normalized().toRotationMatrix();
                const Eigen::Quaterniond rot_quad(cos(M_PI/8.0), 0.0, 0.0, sin(M_PI/8.0)); // Quad mesh is rotated by 45deg
                q_mesh = rot_quad*q_mesh;
                Eigen::Vector3d position = 
                    helper::Point2vec3d(it->vehicle_odom.pose.pose.position);
                
                // Get marker for sphere of influence for each quad
                visualization_functions::SphereMarker(position,
                            frame_id, it->name, sphere_size, sphere_color, 
                            sphere_transparency, 0, &quadArray);

                // Get quad mesh
                visualization_functions::MeshMarker(position, q_mesh,
                            frame_id, it->name, quad_size, it->color, 
                            pos_transparency, 12, &quadArray);

                // Get force arrow around quad
                visualization_functions::ForceMarker(position, 
                            it->force_field, max_acc, frame_id, 
                            it->name, force_color, 13, &quadArray);

                visualization_functions::NameMarker(position, it->name,
                            frame_id, it->name, text_color, 4, &quadArray);

                // Get triad frame arrows
                const double arrowLength = 0.2; 
                visualization_functions::FrameMarker(position, orientation_frame,
                            frame_id, it->name, frame_color, 15, arrowLength, &quadArray);
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