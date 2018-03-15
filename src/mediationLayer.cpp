
#include "mediation_layer/globals.h"

// Global variables--------------------------
globalVariables globals_;
mutexStruct mutexes_;

int main(int argc, char** argv){
  ros::init(argc, argv, "mediation_layer");
  ros::NodeHandle node("~");
  ROS_INFO("Mediation Layer started!");

  // Get arena corners
  std::vector<double> corner1, corner2;
  node.getParam("arena_corner1", corner1);
  node.getParam("arena_corner2", corner2);
  Eigen::Vector3d arena_corner1(corner1[0], corner1[1], corner1[2]);
  Eigen::Vector3d arena_corner2(corner2[0], corner2[1], corner2[2]);

  // Get parameters for the mediation layer ------------------------
  double max_acc, max_vel, d_thresh, d_min, k, kd, k_force;
  node.getParam("max_acceleration", max_acc);
  node.getParam("max_velocity", max_vel);
  node.getParam("dist_threshold", d_thresh);
  node.getParam("dist_minimum", d_min);
  node.getParam("k", k);
  node.getParam("kd", kd);
  node.getParam("k_force", k_force);

  // Get thread parameters ------------------------------------------
  double mediation_layer_rate, rviz_update_rate, game_state_update_rate;
  std::string visualization_topic, game_state_topic;
  node.getParam("mediation_layer_rate", mediation_layer_rate);
  node.getParam("rviz_update_rate", rviz_update_rate);
  node.getParam("game_state_update_rate", game_state_update_rate);
  node.getParam("visualization_topic", visualization_topic);
  node.getParam("game_state_topic", game_state_topic);
  

  // Initialize the Mediation Layer object --------------------------
  globals_.obj_mid_layer = 
  		MediationLayer(visualization_topic, game_state_topic,
                     arena_corner1, arena_corner2, max_acc, max_vel, 
  	                 d_thresh, d_min, k, kd, k_force, &node);

  // Get all quad names and colors ----------------------------------
  std::vector<std::string> quad_names, quad_colors;
  node.getParam("QuadList", quad_names);
  node.getParam("ColorList", quad_colors);
  if (quad_names.size() != quad_colors.size()) {
  	ROS_ERROR("[mediation layer]: Number of quads is different than number of colors!");
  	return 0;
  }

  // Get all topic suffixes -----------------------------------------
  std::string input_ref_topic, input_odom_topic, output_ref_topic;
  node.getParam("input_ref_topic", input_ref_topic);
  node.getParam("input_odom_topic", input_odom_topic);
  node.getParam("output_ref_topic", output_ref_topic);

  // Add quads to mediation layer and create subscribers for them
  std::vector<ros::Subscriber> subsPVA;
  std::string sub_topic_name;
  for(uint i = 0; i < quad_names.size(); i++)
  {
  	// Add quad to the mediation layer
  	std::string output_topic;
  	output_topic = "/" + quad_names[i] + output_ref_topic;
  	globals_.obj_mid_layer.AddQuad(quad_names[i], quad_colors[i],
  		                           output_topic, &node);

  	// Set subscriber to get references to the quad
    sub_topic_name = "/" + quad_names[i] + input_ref_topic;
    ROS_INFO("[mediation layer]: Subscribing to: %s", sub_topic_name.c_str());
    subsPVA.push_back(node.subscribe<mg_msgs::PVA>
    	(sub_topic_name, 10, boost::bind(callbacks::PVACallback, _1, quad_names[i])));

    // Set subscriber to get position measurements from the quads
    sub_topic_name = "/" + quad_names[i] + input_odom_topic;
    ROS_INFO("[mediation layer]: Subscribing to: %s", sub_topic_name.c_str());
    subsPVA.push_back(node.subscribe<nav_msgs::Odometry>
    	(sub_topic_name, 10, boost::bind(callbacks::OdomCallback, _1, quad_names[i])));
  }

  // Threads -------------------------------------------
  std::thread h_mediation_layer_thread, h_visualization_thread;
  std::thread h_static_visualization_thread, h_heartbeat_thread;
  std::thread h_game_state_pub_thread;
  h_mediation_layer_thread = std::thread(threads::MediationLayerThread, mediation_layer_rate);
  h_visualization_thread = std::thread(threads::VisualizationThread, rviz_update_rate);
  h_static_visualization_thread = std::thread(threads::StaticObjectsVisualizationThread);
  h_heartbeat_thread = std::thread(threads::HeartbeatThread);
  h_game_state_pub_thread = std::thread(threads::GameStatePubThread, game_state_update_rate);

  // ROS loop that starts callbacks/publishers
  ros::spin();

  // Kill mutexes
  mutexes_.destroy();

  return 0;
};