

#ifndef RK4_H_
#define RK4_H_

#include <Eigen/Dense>

// ROS message types
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "px4_control/PVA.h"

class rk4 {  // Runge-kutta for time-invariant systems
 public:

 	// Constructors
 	rk4();
 	rk4(const double &k,
 		const double &kd,
        const double &max_vel,
        const double &max_acc);

 	// Methods
 	// void DifferentialEquation(const Eigen::Vector3d &F, 
  //                             const Eigen::VectorXd &state0, 
  //                             Eigen::VectorXd *state_dot);
	void DifferentialEquation(const Eigen::Vector3d &F,
						      const Eigen::VectorXd &state0,
		  				      const px4_control::PVA &Ref,
	                          Eigen::VectorXd *state_dot);
 	// void UpdateStates(const Eigen::Vector3d &F,
  //                     const double &dt);
 	void UpdateStates(const Eigen::Vector3d &F,
 					  const px4_control::PVA &Ref,
                  	  const double &dt);
    // void ResetStates();
    void ResetStates(const px4_control::PVA &Ref);
    void GetPos(Eigen::Vector3d *pos);
    void GetPos(geometry_msgs::Point *pos);
    void GetVel(Eigen::Vector3d *vel);
    void GetVel(geometry_msgs::Vector3 *vel);
    void GetAcc(Eigen::Vector3d *acc);
    void GetAcc(geometry_msgs::Vector3 *acc);

 private:
    // Eigen::VectorXd state_;
    // Eigen::Vector3d accel_;
    // Eigen::Vector3d e_;		 // Error vector
    // Eigen::Vector3d e_dot_;	 // Error derivative
    Eigen::Vector3d y_;		 // Integrated position
    Eigen::Vector3d y_dot_;	 // Integrated velocity
    Eigen::Vector3d y_ddot_; // Acceleration
    Eigen::Matrix3d K_;
    Eigen::Matrix3d Kd_;
    double max_acc_;
    double max_vel_;
    // Eigen::MatrixXd A_;
    // Eigen::MatrixXd B_;
    // uint x_dim_;
    // uint u_dim_;
};


#endif  // ML_CLASS_H_