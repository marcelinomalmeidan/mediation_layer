// Cpp libraries
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <thread>

// Defined libraries
#include "mediation_layer/ml_class.h"
#include "mediation_layer/balloon_class.h"
#include "mediation_layer/threads.h"
#include "mediation_layer/callbacks.h"
#include "mediation_layer/services.h"
#include "mediation_layer/helper.h"

// Structs for global variables
#include "mediation_layer/structs.h"

// ROS message types
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "mg_msgs/PVA.h"
#include "mg_msgs/GameState.h"
#include "mg_msgs/SetQuadBool.h"

// ROS Service types
#include "std_srvs/SetBool.h"


// Declare global variables
extern globalVariables globals_;
extern mutexStruct mutexes_;