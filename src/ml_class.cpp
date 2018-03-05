#include "mediation_layer/ml_class.h"

// Runge-kutta class ----------------------------------------------
rk4::rk4() {

}

rk4::rk4(const double &k, const double &kd,
         const double &max_vel, const double &max_acc) {
	x_dim_ = 6;
	u_dim_ = 3;
	state_ = Eigen::MatrixXd::Zero(x_dim_, 1);
	accel_ = Eigen::Vector3d::Zero();
	max_acc_ = max_acc;
	max_vel_ = max_vel;
	A_ = Eigen::MatrixXd::Zero(x_dim_, x_dim_);
	B_ = Eigen::MatrixXd::Zero(x_dim_, u_dim_);

	for(uint i = 0; i < 3; i++) {
		A_(i,i+3) = 1;
		A_(i+3,i) = -k;
		A_(i+3,i+3) = -kd;
		B_(i+3,i) = -1;
	}
	// std::cout << "A:\n" << A << std::endl;
	// std::cout << "B:\n" << B << std::endl;
}

void rk4::DifferentialEquation(const Eigen::Vector3d &F,
							   const Eigen::VectorXd &state0,
	                           Eigen::VectorXd *state_dot) {
	Eigen::VectorXd x_dot = A_*state0 + B_*F;
	Eigen::Vector3d vel(x_dot[0], x_dot[1], x_dot[2]);
	Eigen::Vector3d acc(x_dot[3], x_dot[4], x_dot[5]);

	// Cap acceleration to maximum value
	if(acc.norm() > max_acc_) {
		acc = max_acc_*acc.normalized();
		x_dot[3] = acc[0];
		x_dot[4] = acc[1];
		x_dot[5] = acc[2];
	}

	// If velocity is above norm, we only allow accelerations that
	// will reduce the velocity (or rotate it)
	if(vel.norm() >= max_vel_) {
		// Project acceleration into velocity
		const double projection = acc.dot(vel);

		// If projection is positive, we only allow acc to rotate velocity
		if(projection > 0) {
			acc = acc - projection*acc;  // Remove the component along vel
			x_dot[3] = acc[0];
			x_dot[4] = acc[1];
			x_dot[5] = acc[2];
		}
	}

	*state_dot = x_dot;
}

void rk4::UpdateStates(const Eigen::Vector3d &F,
	                   const double &dt) {
	const double dt_half = dt/2.0;
	Eigen::VectorXd k1(6), k2(6), k3(6), k4(6);

	this->DifferentialEquation(F, state_, &k1);
	this->DifferentialEquation(F, state_ + dt_half*k1, &k2);
	this->DifferentialEquation(F, state_ + dt_half*k2, &k3);
	this->DifferentialEquation(F, state_ + dt*k3, &k4);

	Eigen::VectorXd deltaX = dt*(k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
	state_ = state_ + deltaX;
	accel_ = Eigen::Vector3d(k1[3], k1[4], k1[5]);
}

void rk4::ResetStates() {
	state_ = Eigen::MatrixXd::Zero(x_dim_, 1);
	accel_ = Eigen::Vector3d::Zero();
}

void rk4::GetPos(Eigen::Vector3d *pos) {
	*pos = Eigen::Vector3d(state_[0], state_[1], state_[2]);
}

void rk4::GetPos(geometry_msgs::Point *pos) {
	Eigen::Vector3d eigen_pos;
	this->GetPos(&eigen_pos);
	pos->x = eigen_pos[0];
	pos->y = eigen_pos[1];
	pos->z = eigen_pos[2];
}

void rk4::GetVel(Eigen::Vector3d *vel) {
	*vel = Eigen::Vector3d(state_[3], state_[4], state_[5]);
}

void rk4::GetVel(geometry_msgs::Vector3 *vel) {
	Eigen::Vector3d eigen_vel;
	this->GetVel(&eigen_vel);
	vel->x = eigen_vel[0];
	vel->y = eigen_vel[1];
	vel->z = eigen_vel[2];
}

void rk4::GetAcc(Eigen::Vector3d *acc) {
	*acc = accel_;
}

void rk4::GetAcc(geometry_msgs::Vector3 *acc) {
	Eigen::Vector3d eigen_acc;
	this->GetAcc(&eigen_acc);
	acc->x = eigen_acc[0];
	acc->y = eigen_acc[1];
	acc->z = eigen_acc[2];
}


// Mediation layer class ------------------------------------------
MediationLayer::MediationLayer() {
	n_quads_ = 0;
}

MediationLayer::MediationLayer(const std::string &visualization_topic,
			                   const Eigen::Vector3d arena_corner1,
			                   const Eigen::Vector3d arena_corner2,
	                           ros::NodeHandle *nh) {
	pub_vis_ = nh->advertise
		<visualization_msgs::MarkerArray>(visualization_topic, 10);	
	std::string arena_name = "arena";
	arena_box_ = BoxPlanes(arena_corner1, arena_corner2, arena_name);
	n_quads_ = 0;
}

void MediationLayer::PrintQuadNames() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		std::cout << it->name << ";" << std::endl;
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
		ROS_INFO("Couldn't print reference: quad name not found");
	}
}

void MediationLayer::AddQuad(const std::string &quad_name,
                 			 const std::string &output_topic,
                 			 ros::NodeHandle *nh) {
	QuadData new_quad;
	new_quad.name = quad_name;

	// Check whether quad name already exists
	if(quads_.find(new_quad) != quads_.end()) {
		ROS_WARN("Tried to add quad ""%s"": already exists!", quad_name.c_str());
	} else {
		px4_control::PVA emptyPVA = helper::GetEmptyPVA();
		new_quad.reference = emptyPVA;
		new_quad.ml_reference = emptyPVA;
		new_quad.force_field = Eigen::Vector3d(0.0, 0.0, 0.0);
		new_quad.is_active = false;
		new_quad.error_integrator = rk4(4.0, 3.0, max_vel_, max_acc_);
		new_quad.nh = *nh;
		new_quad.pub_mediation_layer = new_quad.nh.advertise<px4_control::PVA>(output_topic, 1);
		visualization_functions::SelectColor(n_quads_, &new_quad.color);
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
             			 				 const px4_control::PVA &reference) {
	std::set<QuadData>::iterator it;
	this->FindQuadIndex(name, &it);
	if (it != quads_.end()) {
		it->reference = reference;
		
		// If it was inactive, reset the ml_reference
		if(!it->is_active) {
			it->ml_reference = reference;
			if(!it->is_active) {
				it->error_integrator.ResetStates();
			}
			it->is_active = true;
		}
	} else {
		ROS_INFO("Couldn't update reference: quad %s not found", name.c_str());
	}
}

void MediationLayer::UpdateVehicleReactionForces() {

	static double k_force = 3;
    static double f_max = 1000.0;
    const Eigen::Vector3d z_axis(0.0, 0.0, 1.0);

     // Initialize relative forces to zero
    std::set<QuadData>::iterator it1, it2;
    for(it1 = quads_.begin(); it1 != quads_.end(); ++it1) {
    	it1->force_field = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

	for(it1 = quads_.begin(); it1 != quads_.end(); ++it1) {
		
		if(!it1->is_active) {
			continue;
		}

		// Update the force field to react to other vehicles
		for(it2 = quads_.begin(); it2 != quads_.end(); ++it2) {
			if (it1 == it2) {
				continue;
				// ROS_INFO("same");
			}

			if(!it2->is_active) {
				continue;
			}

			// Get distance magnitude and direction
			Eigen::Vector3d dist_vec = 
					helper::Point2vec3d(it1->ml_reference.Pos) - 
					helper::Point2vec3d(it2->ml_reference.Pos);
			double norm_dist = dist_vec.norm();
			Eigen::Vector3d dist_direction = dist_vec.normalized();
			//Get reacting forces
			if (norm_dist < d_thresh_) {
				double force_magnitude;
				if (norm_dist <= d_min_) {  // Avoid calculating negative forces
					force_magnitude = f_max;
				} else {
					force_magnitude = k_force*(d_thresh_-norm_dist)/(norm_dist-d_min_);
				}
				force_magnitude = std::min(f_max, force_magnitude);

				// Get perpendicular direction
				Eigen::Vector3d dir_perp = (z_axis.cross(dist_direction)).normalized();

				// std::cout << force_magnitude << std::endl;
				it1->force_field = it1->force_field + force_magnitude*dist_direction;
				                                    // + force_magnitude*dir_perp;
				// it2->force_field = it2->force_field - force_magnitude*dist_direction;
			}
		}
	}
}

void MediationLayer::UpdateArenaReactionForces() {

	static double k_force = 3;
    static double f_max = 1000.0;

    std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		
		if(!it->is_active) {
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
			if (norm_dist < d_thresh_) {
				double force_magnitude;
				if (norm_dist <= d_min_) {  // Avoid calculating negative forces
					force_magnitude = f_max;
				} else {
					force_magnitude = k_force*(d_thresh_-norm_dist)/(norm_dist-d_min_);
				}
				force_magnitude = std::min(f_max, force_magnitude);
				it->force_field = it->force_field + force_magnitude*normal[i];
			}
		}
	}
}

void MediationLayer::UpdateMediationLayerOutputs(const double &dt) {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {

			// Update the error dynamics
			it->error_integrator.UpdateStates(it->force_field, dt);

			// Get errors from the error_integrator
			geometry_msgs::Point error;
			geometry_msgs::Vector3 error_dot, error_ddot;
			it->error_integrator.GetPos(&error);
			it->error_integrator.GetVel(&error_dot);
			it->error_integrator.GetAcc(&error_ddot);

			// Populate structure for new reference data
			px4_control::PVA ml_reference;
			
			ml_reference.Pos = 
				helper::SubtractPoint(it->reference.Pos, error);
			ml_reference.Vel = 
				helper::SubtractVector3(it->reference.Vel, error_dot);
			ml_reference.Acc = 
				helper::SubtractVector3(it->reference.Acc, error_ddot);
			ml_reference.yaw = it->reference.yaw;

			// Set new reference data
			it->ml_reference = ml_reference;
		}
}

void MediationLayer::PublishMLReferences() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		// ROS_INFO("Publishing references for quad %s", it->name.c_str());
		it->pub_mediation_layer.publish(it->ml_reference);
	}
}

void MediationLayer::VisualizeArena() {
	visualization_msgs::MarkerArray markers_arena;
	std::string frame_id = "world";
	arena_box_.VisualizeBox(frame_id, &markers_arena);

	pub_vis_.publish(markers_arena);
}