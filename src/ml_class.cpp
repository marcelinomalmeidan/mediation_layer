#include "mediation_layer/ml_class.h"


// Mediation layer class ------------------------------------------
MediationLayer::MediationLayer() {
	n_quads_ = 0;
}

MediationLayer::MediationLayer(const std::string &visualization_topic,
							   const std::string &game_state_topic,
			                   const Eigen::Vector3d &arena_corner1,
			                   const Eigen::Vector3d &arena_corner2,
			                   const double &max_acc,
			                   const double &max_vel,
			                   const double &d_thresh,
			                   const double &d_min,
			                   const double &k,
			                   const double &kd,
			                   const double &k_force,
	                           ros::NodeHandle *nh) {
	max_acc_ = max_acc;
	max_vel_ = max_vel;
	d_thresh_ = d_thresh;
	d_min_ = d_min;
	k_ = k;
	kd_ = kd;
	k_force_ = k_force;
	n_quads_ = 0;
	pub_vis_ = nh->advertise
		<visualization_msgs::MarkerArray>(visualization_topic, 1);	
	pub_game_state_ = nh->advertise<mg_msgs::GameState>(game_state_topic, 1);
	std::string arena_name = "arena";
	arena_box_ = BoxPlanes(arena_corner1, arena_corner2, arena_name);
}

void MediationLayer::PrintQuadNames() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		std::cout << it->name << std::endl;
	}
	std::cout << std::endl;
}
void MediationLayer::PrintQuadReferences(const std::string &name) {
	std::set<QuadData>::iterator it;
	this->FindQuadIndex(name, &it);
	if (it != quads_.end()) {
		ROS_INFO("Quad %s references: %f\t%f\t%f", name.c_str(), 
			     it->reference.Pos.x, it->reference.Pos.y, it->reference.Pos.z);
	} else {
		ROS_INFO("[mediation layer] Couldn't print reference: quad name not found");
	}
}

void MediationLayer::AddQuad(const std::string &quad_name,
                 			 const std::string &quad_color,
                 			 const bool has_shield,
                 			 const std::string &output_topic,
                 			 ros::NodeHandle *nh) {
	QuadData new_quad;
	new_quad.name = quad_name;

	// Check whether quad name already exists
	if(quads_.find(new_quad) != quads_.end()) {
		ROS_WARN("[mediation layer] Tried to add quad ""%s"": already exists!", quad_name.c_str());
	} else {
		mg_msgs::PVA emptyPVA = helper::GetEmptyPVA();
		new_quad.reference = emptyPVA;
		new_quad.ml_reference = emptyPVA;
		new_quad.vehicle_odom = helper::GetZeroOdom();
		new_quad.force_field = Eigen::Vector3d(0.0, 0.0, 0.0);
		new_quad.has_shield = has_shield;
		new_quad.ref_is_active = false;
		new_quad.odom_is_active = false;
		new_quad.last_reference_stamp = ros::Time::now();
		new_quad.last_measurement_stamp = ros::Time::now();
		new_quad.error_integrator = rk4(k_, kd_, max_vel_, max_acc_);
		new_quad.nh = *nh;
		new_quad.pub_mediation_layer = new_quad.nh.advertise<mg_msgs::PVA>(output_topic, 1);
		if(quad_color.compare("any") == 0) {
			visualization_functions::SelectColor(n_quads_, &new_quad.color);
		}else {
			visualization_functions::SelectColor(quad_color, &new_quad.color);
		}
		quads_.insert(new_quad);
		n_quads_ = n_quads_ + 1;
	}
}

void MediationLayer::FindQuadIndex(const std::string &name,
	                               std::set<QuadData>::iterator *index) {
	QuadData quad_with_name;
	quad_with_name.name = name;
	*index = quads_.find(quad_with_name);
}

void MediationLayer::UpdateQuadReference(const std::string &name, 
             			 				 const mg_msgs::PVA &reference) {
	std::set<QuadData>::iterator it;
	this->FindQuadIndex(name, &it);
	if (it != quads_.end()) {
		it->reference = reference;
		it->last_reference_stamp = ros::Time::now();
		
		// If it is inactive, set it to active
		if(!it->ref_is_active) {
			it->ref_is_active = true;
			if(!it->odom_is_active) {
				it->error_integrator.ResetStates(reference);
			}
		}
	} else {
		ROS_INFO("[mediation layer] Couldn't update reference: quad %s not found", name.c_str());
	}
}

void MediationLayer::UpdateQuadOdom(const std::string &name, 
                                    const nav_msgs::Odometry &odom) {
	std::set<QuadData>::iterator it;
	this->FindQuadIndex(name, &it);
	if (it != quads_.end()) {
		it->vehicle_odom = odom;
		it->last_measurement_stamp = ros::Time::now();

		// If odom is inactive, activate it
		if(!it->odom_is_active) {
			it->odom_is_active = true;
		}

		// If references are inactive, reset the ml_reference to the current position
		if(!it->ref_is_active) {
			it->error_integrator.ResetStates(odom);
		}
	} else {
		ROS_INFO("[mediation layer] Couldn't update reference: quad %s not found", name.c_str());
	}
}

void MediationLayer::ResetForces() {
    // Initialize relative forces to zero
    std::set<QuadData>::iterator it;
    for(it = quads_.begin(); it != quads_.end(); ++it) {
    	it->force_field = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

}

void MediationLayer::UpdateVehicleReactionForces() {

	// static double k_force = 3;
    static double f_max = 1000.0;
    const Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    const double epsilon = 0.001;

    std::set<QuadData>::iterator it1, it2;
	for(it1 = quads_.begin(); it1 != quads_.end(); ++it1) {
		
		if(!it1->odom_is_active) {
			continue;
		}

		// Update the force field to react to other vehicles
		for(it2 = it1; it2 != quads_.end(); ++it2) {
			if (it1 == it2) {
				continue;
			}

			if(!it2->odom_is_active) {
				continue;
			}

			// Get positions of quads
			const Eigen::Vector3d pos1 = 
				helper::Point2vec3d(it1->vehicle_odom.pose.pose.position);
			const Eigen::Vector3d pos2 = 
				helper::Point2vec3d(it2->vehicle_odom.pose.pose.position);

			// Get distance magnitude and direction
			Eigen::Vector3d dist_vec = pos1 - pos2;
			double norm_dist = dist_vec.norm();

			if (norm_dist > d_thresh_) {
				continue;  // Quads too far appart to interact
			}

			// Calculate magnitude of reacting force
			double force_magnitude;
			if (norm_dist <= d_min_) {  // Avoid calculating negative forces
				force_magnitude = f_max;
			} else {
				force_magnitude = k_force_*(d_thresh_-norm_dist)/(d_thresh_-d_min_);
			}
			force_magnitude = std::min(f_max, force_magnitude);

			// Shield reaction ----------------------------------------
			bool shield_active = false;
			if(it1->has_shield) {
				// Add shield reaction
				const double yaw = it1->ml_reference.yaw;
				Eigen::Vector3d plane_normal(cos(yaw), sin(yaw), 0.0);
				Eigen::Vector3d plane_origin = pos1;
				Eigen::Vector3d z_plane = z_axis;
				Eigen::Vector3d y_plane = z_plane.cross(plane_normal);
				double width = d_thresh_, height = d_thresh_;

				Wall shield_plane(plane_origin, plane_normal, z_plane, y_plane,
					              width, height);

				// Get nearest point in the shield
				double plane_dist;
				shield_plane.DistancePoint2WallPlane(pos2, &plane_dist);
				// ROS_INFO("Projection in wall: %d, distance: %f", inWall, plane_dist);
				if(plane_dist > 0) {  // If in front of shield
					// Wall reaction
					it1->force_field = it1->force_field - force_magnitude*shield_plane.plane_.normal_;
					it2->force_field = it2->force_field + force_magnitude*shield_plane.plane_.normal_;
					shield_active = true;
				}
			}

			// See if other vehicle has shield (react to it)
			if(it2->has_shield) {
				// Add shield reaction
				const double yaw = it2->ml_reference.yaw;
				Eigen::Vector3d plane_normal(cos(yaw), sin(yaw), 0.0);
				Eigen::Vector3d plane_origin = pos2;
				Eigen::Vector3d z_plane = z_axis;
				Eigen::Vector3d y_plane = z_plane.cross(plane_normal);
				double width = d_thresh_, height = d_thresh_;

				Wall shield_plane(plane_origin, plane_normal, z_plane, y_plane,
					              width, height);

				// Get nearest point in the shield
				double plane_dist;
				shield_plane.DistancePoint2WallPlane(pos2, &plane_dist);
				if(plane_dist > 0) {  // If in front of shield
					// Wall reaction
					it1->force_field = it1->force_field + force_magnitude*shield_plane.plane_.normal_;
					it2->force_field = it2->force_field - force_magnitude*shield_plane.plane_.normal_;
					shield_active = true;
				}
			}


			// Calculate sphere reaction ---------------------------------------------
			// Sphere reaction (only happens when there was no shield reaction)
			if(shield_active) {
				continue;
			}

			// Get direction of force
			Eigen::Vector3d dist_direction;
			if(norm_dist > epsilon){
				dist_direction = dist_vec.normalized();
			} else {
				dist_direction = Eigen::Vector3d(0.0, 0.0, 0.0);
			}

			//Get reacting forces
			it1->force_field = it1->force_field + force_magnitude*dist_direction;
			it2->force_field = it2->force_field - force_magnitude*dist_direction;
		}
	}
}

void MediationLayer::UpdateArenaReactionForces() {

	// static double k_force = 3;
    static double f_max = 1000.0;

    std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		
		if(!it->odom_is_active) {
			continue;
		}

		// Get distance between quad and all planes
		std::vector<double> dist;
		std::vector<Eigen::Vector3d> normal;
		arena_box_.DistancePoint2Planes(
				helper::Point2vec3d(it->ml_reference.Pos),
                &dist, &normal);

		// Calculate reaction forces between quad and planes
		for (uint i = 0; i < dist.size(); i++) {
			const double norm_dist = dist[i];
			// std::cout << "quad dist " << i << ": " << norm_dist << std::endl;
			if (norm_dist < d_thresh_) {
				double force_magnitude;
				if (norm_dist <= d_min_) {  // Avoid calculating negative forces
					force_magnitude = f_max;
				} else {
					force_magnitude = k_force_*(d_thresh_-norm_dist)/(norm_dist-d_min_);
				}
				force_magnitude = std::min(f_max, force_magnitude);
				it->force_field = it->force_field + force_magnitude*normal[i];
				// std::cout << force_magnitude*normal[i][0] << " "
				// 		  << force_magnitude*normal[i][1] << " "
				// 		  << force_magnitude*normal[i][2] << std::endl;
			}
		}
	}
}

void MediationLayer::UpdateMediationLayerOutputs(const double &dt) {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {

		if(!it->ref_is_active) {
			continue;
		}

		// Update the error dynamics
		it->error_integrator.UpdateStates(it->force_field, 
			                              it->reference, dt);

		// Get outputs from the error_integrator
		geometry_msgs::Point pos;
		geometry_msgs::Vector3 pos_dot, pos_ddot;
		it->error_integrator.GetPos(&pos);
		it->error_integrator.GetVel(&pos_dot);
		it->error_integrator.GetAcc(&pos_ddot);

		// Populate structure for new reference data
		mg_msgs::PVA ml_reference;
		
		ml_reference.Pos = pos;
		ml_reference.Vel = pos_dot;
		ml_reference.Acc = pos_ddot;
		ml_reference.yaw = it->reference.yaw;

		// Set new reference data
		it->ml_reference = ml_reference;
	}
}

void MediationLayer::PublishMLReferences() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		if(it->ref_is_active) {
			it->pub_mediation_layer.publish(it->ml_reference);	
		}
	}
}

void MediationLayer::GetQuadPositions(std::vector<Eigen::Vector3d> *quad_positions) {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		if(it->ref_is_active) {
			Eigen::Vector3d pos(helper::Point2vec3d(it->ml_reference.Pos));
			quad_positions->push_back(pos);
		}
	}
}

void MediationLayer::VisualizeArena() {
	visualization_msgs::MarkerArray markers_arena;
	std::string frame_id = "world";
	arena_box_.VisualizeBox(frame_id, &markers_arena);

	pub_vis_.publish(markers_arena);
}