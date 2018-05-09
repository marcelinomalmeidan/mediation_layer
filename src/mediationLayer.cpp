
#include "mediation_layer/globals.h"

// Global variables--------------------------
globalVariables globals_;
mutexStruct mutexes_;

int main(int argc, char** argv){
  ros::init(argc, argv, "mediation_layer");
  ros::NodeHandle node("~");
  ROS_INFO("Mediation Layer started!");

  // Get arena corners ---------------------------------------------
  std::vector<double> corner1, corner2;
  node.getParam("arena_corner1", corner1);
  node.getParam("arena_corner2", corner2);
  Eigen::Vector3d arena_corner1(corner1[0], corner1[1], corner1[2]);
  Eigen::Vector3d arena_corner2(corner2[0], corner2[1], corner2[2]);

  // Get balloons --------------------------------------------------
  double pop_distance = 0.25;
  globals_.balloons = BalloonSet(pop_distance, &node);
  std::vector<double> balloon_positions;
  std::vector<std::string> balloon_colors;
  node.getParam("BalloonPosition", balloon_positions);
  node.getParam("BalloonColors", balloon_colors);
  if (float(balloon_colors.size()) > float(balloon_positions.size())/3.0) {
    ROS_ERROR("[mediation layer]: Balloon positions not well defined!");
    return 0;
  } else {
    // Add balloons
    for (uint i = 0; i < balloon_colors.size(); i++) {
      std_msgs::ColorRGBA color;
      visualization_functions::SelectColor(balloon_colors[i], &color);
      Eigen::Vector3d pos(balloon_positions[3*i],
                          balloon_positions[3*i + 1],
                          balloon_positions[3*i + 2]);
      globals_.balloons.AddBalloon(balloon_colors[i], color, pos);
      ROS_INFO("[mediation layer]: Added a %s balloon at position %f %f %f",
                balloon_colors[i].c_str(), pos[0], pos[1], pos[2]);
    }
  }

  // Get parameters for the mediation layer ------------------------
  double max_acc, max_vel, max_in_acc, max_in_vel, 
         d_thresh, d_min, k, kd, k_force;
  node.getParam("max_acceleration", max_acc);
  node.getParam("max_velocity", max_vel);
  node.getParam("max_input_acceleration", max_in_acc);
  node.getParam("max_input_velocity", max_in_vel);
  node.getParam("dist_threshold", d_thresh);
  node.getParam("dist_minimum", d_min);
  node.getParam("k", k);
  node.getParam("kd", kd);
  node.getParam("k_force", k_force);

  // Get thread parameters ------------------------------------------
  double mediation_layer_rate, rviz_update_rate, 
         game_state_update_rate, tf_update_rate;
  std::string visualization_topic, game_state_topic, game_start_topic;
  node.getParam("mediation_layer_rate", mediation_layer_rate);
  node.getParam("rviz_update_rate", rviz_update_rate);
  node.getParam("game_state_update_rate", game_state_update_rate);
  node.getParam("tf_update_rate", tf_update_rate);
  node.getParam("visualization_topic", visualization_topic);
  node.getParam("game_state_topic", game_state_topic);
  node.getParam("game_start_topic", game_start_topic);
  

  // Initialize the Mediation Layer object --------------------------
  globals_.obj_mid_layer = 
  		MediationLayer(visualization_topic, game_state_topic, game_start_topic,
                     arena_corner1, arena_corner2, max_acc, max_vel, 
  	                  max_in_acc, max_in_vel, d_thresh, d_min,
                      k, kd, k_force, &node);

  // Add cylindrical obstacles to the mediation layer
  globals_.obj_mid_layer.AddBalloonRodObstacles(globals_.balloons);

  // Get all quad names, colors and shields -------------------------
  std::vector<std::string> quad_names, quad_colors;
  std::vector<bool> has_shield;
  node.getParam("QuadList", quad_names);
  node.getParam("ColorList", quad_colors);
  node.getParam("HasShield", has_shield);
  if (quad_names.size() != quad_colors.size()) {
  	ROS_ERROR("[mediation layer]: Number of quads is different than number of colors!");
  	return 0;
  }
  if (quad_names.size() != has_shield.size()) {
    ROS_INFO("%d", int(has_shield.size()));
    ROS_ERROR("[mediation layer]: Number of quads is different than shield assignment!");
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
  		                             has_shield[i], output_topic, &node);

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

  // Services ------------------------------------------
  ros::ServiceServer shield_srv = node.advertiseService
       ("/mediation_layer/set_quad_shield", services::SetShield);
  ros::ServiceServer ready_srv = node.advertiseService
       ("/mediation_layer/set_quad_ready", services::ReadyForGame);
  ros::ServiceServer takeoff_land_srv = node.advertiseService
       ("/mediation_layer/request_to_land_quad", services::SetTakeOffLanding);

  // Callbacks -----------------------------------------
  ros::Subscriber land_sub = node.subscribe
       ("/mediation_layer/land_quads", 10, callbacks::LandCallback);

  // Threads -------------------------------------------
  std::thread h_mediation_layer_thread, h_visualization_thread;
  std::thread h_static_visualization_thread, h_heartbeat_thread;
  std::thread h_game_state_pub_thread, h_tf_thread;
  h_mediation_layer_thread = std::thread(threads::MediationLayerThread, mediation_layer_rate);
  h_visualization_thread = std::thread(threads::VisualizationThread, rviz_update_rate);
  h_static_visualization_thread = std::thread(threads::StaticObjectsVisualizationThread);
  h_heartbeat_thread = std::thread(threads::HeartbeatThread);
  h_game_state_pub_thread = std::thread(threads::GameStatePubThread, game_state_update_rate);
  h_tf_thread = std::thread(threads::tfThread, tf_update_rate);

  // ROS loop that starts callbacks/publishers
  ros::spin();

  // Kill mutexes
  mutexes_.destroy();

  return 0;
};