#include "mediation_layer/rk4.h"

// Runge-kutta class ----------------------------------------------
rk4::rk4() {

}

rk4::rk4(const double &k, const double &kd,
         const double &max_vel, const double &max_acc,
         const double &max_input_acc) {
	// x_dim_ = 6;
	// u_dim_ = 3;
	// state_ = Eigen::MatrixXd::Zero(x_dim_, 1);
	// accel_ = Eigen::Vector3d::Zero();
	// A_ = Eigen::MatrixXd::Zero(x_dim_, x_dim_);
	// B_ = Eigen::MatrixXd::Zero(x_dim_, u_dim_);

	// for(uint i = 0; i < 3; i++) {
	// 	A_(i,i+3) = 1;
	// 	A_(i+3,i) = -k;
	// 	A_(i+3,i+3) = -kd;
	// 	B_(i+3,i) = -1;
	// }

	max_acc_ = max_acc;
	max_vel_ = max_vel;
	max_input_acc_ = max_input_acc;
	K_ = k*Eigen::Matrix3d::Identity();
	Kd_ = kd*Eigen::Matrix3d::Identity();
	y_ = Eigen::Vector3d::Zero();
	y_dot_ = Eigen::Vector3d::Zero();
	y_ddot_ = Eigen::Vector3d::Zero();
	max_yaw_dot_ = 1.5;
	// std::cout << "A:\n" << A << std::endl;
	// std::cout << "B:\n" << B << std::endl;
}

// void rk4::DifferentialEquation(const Eigen::Vector3d &F,
// 							   const Eigen::VectorXd &state0,
// 	                           Eigen::VectorXd *state_dot) {
// 	Eigen::VectorXd x_dot = A_*state0 + B_*F;
// 	Eigen::Vector3d vel(x_dot[0], x_dot[1], x_dot[2]);
// 	Eigen::Vector3d acc(x_dot[3], x_dot[4], x_dot[5]);

// 	// Cap acceleration to maximum value
// 	if(acc.norm() > max_acc_) {
// 		acc = max_acc_*acc.normalized();
// 		x_dot[3] = acc[0];
// 		x_dot[4] = acc[1];
// 		x_dot[5] = acc[2];
// 	}

// 	// If velocity is above norm, we only allow accelerations that
// 	// will reduce the velocity (or rotate it)
// 	if(vel.norm() >= max_vel_) {
// 		// Project acceleration into velocity
// 		const double projection = acc.dot(vel);

// 		// If projection is positive, we only allow acc to rotate velocity
// 		if(projection > 0) {
// 			acc = acc - projection*acc;  // Remove the component along vel
// 			x_dot[3] = acc[0];
// 			x_dot[4] = acc[1];
// 			x_dot[5] = acc[2];
// 		}
// 	}

// 	*state_dot = x_dot;
// }

// void rk4::UpdateStates(const Eigen::Vector3d &F,
// 	                   const double &dt) {
// 	const double dt_half = dt/2.0;
// 	Eigen::VectorXd k1(6), k2(6), k3(6), k4(6);

// 	this->DifferentialEquation(F, state_, &k1);
// 	this->DifferentialEquation(F, state_ + dt_half*k1, &k2);
// 	this->DifferentialEquation(F, state_ + dt_half*k2, &k3);
// 	this->DifferentialEquation(F, state_ + dt*k3, &k4);

// 	Eigen::VectorXd deltaX = dt*(k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
// 	state_ = state_ + deltaX;
// 	accel_ = Eigen::Vector3d(k1[3], k1[4], k1[5]);
// }

void rk4::DifferentialEquation(const Eigen::Vector3d &F,
							   const Eigen::VectorXd &state0,
							   const mg_msgs::PVA &Ref,
	                           Eigen::VectorXd *state_dot) {
	// Get references
	Eigen::Vector3d yd, yd_dot, yd_ddot;
	yd << Ref.Pos.x, Ref.Pos.y, Ref.Pos.z;
	yd_dot << Ref.Vel.x, Ref.Vel.y, Ref.Vel.z;
	yd_ddot << Ref.Acc.x, Ref.Acc.y, Ref.Acc.z;

	// Differential equation
	Eigen::Vector3d y, y_dot, y_ddot, e, e_dot;
	y << state0[0], state0[1], state0[2];
	y_dot << state0[3], state0[4], state0[5];
	e = yd - y;
	e_dot = yd_dot - y_dot;

	Eigen::Vector3d input_acc = yd_ddot + K_*e + Kd_*e_dot;
	if(input_acc.norm() > max_input_acc_) {
		input_acc = max_input_acc_*input_acc/input_acc.norm();
	}

	y_ddot = input_acc + F;

	// Cap acceleration to maximum value
	if(y_ddot.norm() > max_acc_) {
		y_ddot = max_acc_*y_ddot.normalized();
	}

	// If velocity is above norm, we only allow accelerations that
	// will reduce the velocity (or rotate it)
	if(y_dot.norm() >= max_vel_) {
		// Project acceleration into velocity
		const double projection = y_ddot.dot(y_dot.normalized());

		// If projection is positive, we only allow y_ddot to rotate velocity
		if(projection > 0) {
			y_ddot = y_ddot - projection*y_dot.normalized();  // Remove the component along vel
		}
	}

	Eigen::VectorXd x_dot(6);
	x_dot << y_dot[0], y_dot[1], y_dot[2], y_ddot[0], y_ddot[1], y_ddot[2];

	*state_dot = x_dot;
}

void rk4::DifferentialYawEquation(const double &yaw0,
                                  const double &yaw_ref,
                                  double *yaw_dot) {
	const double k = K_(0,0);

	// Get reference between -pi and pi
	const double ref = fmod(yaw_ref, M_PI);

	double e = yaw_ref - yaw0;
	if (e > M_PI) {
		e = e - 2*M_PI;
	} else if (e < -M_PI) {
		e = e + 2*M_PI;
	}

	double yaw_vel = k*e; 
	if(fabs(yaw_vel) > max_yaw_dot_) {
		yaw_vel = copysign(max_yaw_dot_, yaw_vel);
	}

	*yaw_dot = yaw_vel;
}

void rk4::UpdateStates(const Eigen::Vector3d &F,
					   const mg_msgs::PVA &Ref,
	          	       const double &dt) {
	const double dt_half = dt/2.0;
	Eigen::VectorXd k1(6), k2(6), k3(6), k4(6);

	Eigen::VectorXd state(6);
	state << y_[0], y_[1], y_[2], y_dot_[0], y_dot_[1], y_dot_[2];

	this->DifferentialEquation(F, state, Ref, &k1);
	this->DifferentialEquation(F, state + dt_half*k1, Ref, &k2);
	this->DifferentialEquation(F, state + dt_half*k2, Ref, &k3);
	this->DifferentialEquation(F, state + dt*k3, Ref, &k4);

	Eigen::VectorXd deltaX = dt*(k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
	state = state + deltaX;
	y_ << state[0], state[1], state[2];
	y_dot_ << state[3], state[4], state[5];
	y_ddot_ = Eigen::Vector3d(k1[3], k1[4], k1[5]);

	// Integrate yaw
	double ky1, ky2, ky3, ky4;
	double yaw_ref = Ref.yaw;
	this->DifferentialYawEquation(yaw_, yaw_ref, &ky1);
	this->DifferentialYawEquation(yaw_ + dt_half*ky1, yaw_ref, &ky2);
	this->DifferentialYawEquation(yaw_ + dt_half*ky2, yaw_ref, &ky3);
	this->DifferentialYawEquation(yaw_ + dt*ky3, yaw_ref, &ky4);
	double deltaYaw = dt*(ky1 + 2.0*ky2 + 2.0*ky3 + ky4)/6.0;
	yaw_ = yaw_ + deltaYaw;
	if (yaw_ < -M_PI) {
		yaw_ = yaw_ + 2*M_PI;
	} else if(yaw_ > M_PI) {
		yaw_ = yaw_ - 2*M_PI;
	}
}

// void rk4::ResetStates() {
// 	state_ = Eigen::MatrixXd::Zero(x_dim_, 1);
// 	accel_ = Eigen::Vector3d::Zero();
// }

void rk4::ResetStates(const mg_msgs::PVA &Ref) {
	y_ << Ref.Pos.x, Ref.Pos.y, Ref.Pos.z;
	y_dot_ << Ref.Vel.x, Ref.Vel.y, Ref.Vel.z;
	y_ddot_ << 0.0, 0.0, 0.0;
}

void rk4::ResetStates(const nav_msgs::Odometry &odom) {
	y_ << odom.pose.pose.position.x,
	      odom.pose.pose.position.y,
	      odom.pose.pose.position.z;
	y_dot_ << 0.0, 0.0, 0.0;
	y_ddot_ << 0.0, 0.0, 0.0;
	yaw_ = helper::getHeadingFromQuat(odom.pose.pose.orientation);
}

void rk4::GetPos(Eigen::Vector3d *pos) {
	// *pos = Eigen::Vector3d(state_[0], state_[1], state_[2]);
	*pos = y_;
	// std::cout << y_[0] << " " << y_[1] << " " << y_[2] << std::endl;
}

void rk4::GetPos(geometry_msgs::Point *pos) {
	Eigen::Vector3d eigen_pos;
	this->GetPos(&eigen_pos);
	pos->x = eigen_pos[0];
	pos->y = eigen_pos[1];
	pos->z = eigen_pos[2];
}

void rk4::GetVel(Eigen::Vector3d *vel) {
	// *vel = Eigen::Vector3d(state_[3], state_[4], state_[5]);
	*vel = y_dot_;
}

void rk4::GetVel(geometry_msgs::Vector3 *vel) {
	Eigen::Vector3d eigen_vel;
	this->GetVel(&eigen_vel);
	vel->x = eigen_vel[0];
	vel->y = eigen_vel[1];
	vel->z = eigen_vel[2];
}

void rk4::GetAcc(Eigen::Vector3d *acc) {
	// *acc = accel_;
	*acc = y_ddot_;
}

void rk4::GetAcc(geometry_msgs::Vector3 *acc) {
	Eigen::Vector3d eigen_acc;
	this->GetAcc(&eigen_acc);
	acc->x = eigen_acc[0];
	acc->y = eigen_acc[1];
	acc->z = eigen_acc[2];
}

void rk4::GetYaw(double *yaw) {
	*yaw = yaw_;
}