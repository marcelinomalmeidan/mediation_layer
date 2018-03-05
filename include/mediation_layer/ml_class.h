// Mediation layer class

#ifndef ML_CLASS_H_
#define ML_CLASS_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "mediation_layer/helper.h"
#include "mediation_layer/visualization_functions.h"
#include "mediation_layer/linear_algebra.h"

#include "visualization_msgs/MarkerArray.h"
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
 	void DifferentialEquation(const Eigen::Vector3d &F, 
                              const Eigen::VectorXd &state0, 
                              Eigen::VectorXd *state_dot);
 	void UpdateStates(const Eigen::Vector3d &F,
                      const double &dt);
    void ResetStates();
    void GetPos(Eigen::Vector3d *pos);
    void GetPos(geometry_msgs::Point *pos);
    void GetVel(Eigen::Vector3d *vel);
    void GetVel(geometry_msgs::Vector3 *vel);
    void GetAcc(Eigen::Vector3d *acc);
    void GetAcc(geometry_msgs::Vector3 *acc);

 private:
    Eigen::VectorXd state_;
    Eigen::Vector3d accel_;
    double max_acc_;
    double max_vel_;
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    uint x_dim_;
    uint u_dim_;
};

struct QuadData {
    std::string name;                            // Unique name for vehicle
    mutable px4_control::PVA reference;          // Input reference
    mutable px4_control::PVA ml_reference;       // Output reference
    mutable Eigen::Vector3d force_field;         // Field for saving ML force input
    mutable bool is_active;                      // Flag indicating health
    mutable rk4 error_integrator;                // Runge-kutta dynamics integrator
    mutable ros::NodeHandle nh;                  // ROS Nodehandle
    mutable ros::Publisher pub_mediation_layer;  // Publishes the ML output
    mutable std_msgs::ColorRGBA color;           // Color for visualization

    bool operator<(const QuadData& other) const {
      // return name.compare(other.name);
        int compareResult = name.compare(other.name);
        return (compareResult < 0);
    }
};

class MediationLayer {
 public:
 	std::set<QuadData> quads_;
    BoxPlanes arena_box_;
 	uint n_quads_;
    ros::Publisher pub_vis_;  // Publisher for visualization
    double max_acc_ = 3.5;
    double max_vel_ = 3.0;
    double d_thresh_ = 1.0;   // Distance at which one quad influences another
    double d_min_ = 0.50;     // Minimum distance allowed (infinite repulsion)

    // Constructors
    MediationLayer();
    MediationLayer(const std::string &visualization_topic,
                   const Eigen::Vector3d arena_corner1,
                   const Eigen::Vector3d arena_corner2,
                   ros::NodeHandle *nh);

    // Methods
    void PrintQuadNames();
    void PrintQuadReferences(const std::string &name);
    void AddQuad(const std::string &quad_name,
                 const std::string &output_topic,
                 ros::NodeHandle *nh);
    void FindQuadIndex(const std::string &name,
    	               std::set<QuadData>::iterator *index);  // Returns -1 if it can't find
    void UpdateQuadReference(const std::string &name, 
                 			 const px4_control::PVA &reference);
    void UpdateVehicleReactionForces();
    void UpdateArenaReactionForces();
    void UpdateMediationLayerOutputs(const double &dt);
    void PublishMLReferences();
    void VisualizeArena();

 private:

    // ros::NodeHandle nh_;
};

#endif  // ML_CLASS_H_