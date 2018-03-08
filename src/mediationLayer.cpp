
#include "mediation_layer/globals.h"

// Global variables--------------------------
globalVariables globals_;
mutexStruct mutexes_;

int main(int argc, char** argv){
  ros::init(argc, argv, "mediation_layer");
  ros::NodeHandle node("~");
  ROS_INFO("Mediation Layer started!");

  // Get arena corners
  Eigen::Vector3d arena_corner1(-10.0, -2.5, 0.0);
  Eigen::Vector3d arena_corner2(10.0, 2.5, 4.0);

  globals_.obj_mid_layer = MediationLayer("/Visualization", arena_corner1, 
  	                                      arena_corner2, &node);

  // Get all quad names and create a subscriber for each one of them
  std::vector<std::string> quad_names;
  node.getParam("QuadList", quad_names);
  std::vector<ros::Subscriber> subsPVA;
  std::string sub_topic_name;
  for(uint i = 0; i < quad_names.size(); i++)
  {
  	// Add quad to the mediation layer
  	std::string output_topic, visualization_topic;
  	output_topic = "/" + quad_names[i] + "/px4_control/PVA_Ref_ML";
  	globals_.obj_mid_layer.AddQuad(quad_names[i], output_topic, &node);

  	// Set subscriber to get references to the quad
    sub_topic_name = "/" + quad_names[i] + "/px4_control/PVA_Ref";
    ROS_INFO("[mediation layer]: Subscribing to: %s", sub_topic_name.c_str());
    subsPVA.push_back(node.subscribe<mg_msgs::PVA>
    	(sub_topic_name, 10, boost::bind(callbacks::PVACallback, _1, quad_names[i])));

    // Set subscriber to get position measurements from the quads
    sub_topic_name = "/" + quad_names[i] + "/local_odom";
    ROS_INFO("[mediation layer]: Subscribing to: %s", sub_topic_name.c_str());
    subsPVA.push_back(node.subscribe<nav_msgs::Odometry>
    	(sub_topic_name, 10, boost::bind(callbacks::OdomCallback, _1, quad_names[i])));
  }

  // Threads -------------------------------------------
  std::thread h_midiation_layer_thread, h_visualization_thread;
  std::thread h_static_visualization_thread, h_heartbeat_thread;
  h_midiation_layer_thread = std::thread(threads::MediationLayerThread);
  h_visualization_thread = std::thread(threads::VisualizationThread);
  h_static_visualization_thread = std::thread(threads::StaticObjectsVisualizationThread);
  h_heartbeat_thread = std::thread(threads::HeartbeatThread);

  // ROS loop that starts callbacks/publishers
  ros::spin();

  // Kill mutexes
  mutexes_.destroy();


  return 0;
};