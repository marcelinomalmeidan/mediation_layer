// Mediation layer class

#ifndef ML_CLASS_H_
#define ML_CLASS_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "mediation_layer/helper.h"
#include "mediation_layer/visualization_functions.h"
#include "mediation_layer/linear_algebra.h"
#include "mediation_layer/rk4.h"

#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"

#include "mg_msgs/PVA.h"
#include "mg_msgs/GameState.h"

struct QuadData {
    std::string name;                            // Unique name for vehicle
    mutable mg_msgs::PVA reference;          // Input reference
    mutable mg_msgs::PVA ml_reference;       // Output reference
    mutable nav_msgs::Odometry vehicle_odom;     // Measured odometry
    mutable Eigen::Vector3d force_field;         // Field for saving ML force input
    mutable bool has_shield;                     // Shield for defending quads
    mutable bool ref_is_active;                  // Flag indicating health of references
    mutable bool odom_is_active;                 // Flag indicating health of measurements
    mutable ros::Time last_reference_stamp;      // For heartbeat
    mutable ros::Time last_measurement_stamp;    // For heartbeat
    mutable rk4 error_integrator;                // Runge-kutta dynamics integrator
    mutable ros::NodeHandle nh;                  // ROS Nodehandle
    mutable ros::Publisher pub_mediation_layer;  // Publishes the ML output
    mutable std_msgs::ColorRGBA color;           // Color for visualization

    bool operator<(const QuadData& other) const {
        int compareResult = name.compare(other.name);
        return (compareResult < 0);
    }
};

class MediationLayer {
 public:
 	std::set<QuadData> quads_;
    BoxPlanes arena_box_;
 	uint n_quads_;
    ros::Publisher pub_vis_;        // Publisher for visualization
    ros::Publisher pub_game_state_; // Publisher for game state
    double max_acc_ = 6.0;          // Maximum integrator acceleration
    double max_vel_ = 2.5;          // Maximum integrator velocity
    double max_in_acc_ = 2.5;       // Maximum input acceleration
    double max_in_vel_ = 6.0;       // Maximum input velocity
    double d_thresh_ = 1.0;   // Distance at which one quad influences another
    double d_min_ = 0.50;     // Minimum distance allowed (infinite repulsion)
    double k_ = 4.0;          // Proportional gain in the error integrator
    double kd_ = 3.0;         // Derivative gain in the error integrator
    double k_force_ = 3.0;     // Repulsion force gain

    // Constructors
    MediationLayer();
    MediationLayer(const std::string &visualization_topic,
                   const std::string &game_state_topic,
                   const Eigen::Vector3d &arena_corner1,
                   const Eigen::Vector3d &arena_corner2,
                   const double &max_acc,
                   const double &max_vel,
                   const double &max_in_acc,
                   const double &max_in_vel,
                   const double &d_thresh,
                   const double &d_min,
                   const double &k,
                   const double &kd,
                   const double &k_force,
                   ros::NodeHandle *nh);

    // Methods
    void PrintQuadNames();
    void PrintQuadReferences(const std::string &name);
    void AddQuad(const std::string &quad_name,
                 const std::string &quad_color,
                 const bool has_shield,
                 const std::string &output_topic,
                 ros::NodeHandle *nh);
    void FindQuadIndex(const std::string &name,
    	               std::set<QuadData>::iterator *index);  // Returns -1 if it can't find
    void ResetForces();
    void UpdateQuadReference(const std::string &name, 
                 			 const mg_msgs::PVA &reference);
    void UpdateQuadOdom(const std::string &name, 
                        const nav_msgs::Odometry &odom);
    void UpdateVehicleReactionForces();
    void UpdateArenaReactionForces();
    void UpdateMediationLayerOutputs(const double &dt);
    void PublishMLReferences();
    void GetQuadPositions(std::vector<Eigen::Vector3d> *quad_positions);
    void VisualizeArena();

 private:

    // ros::NodeHandle nh_;
};

#endif  // ML_CLASS_H_