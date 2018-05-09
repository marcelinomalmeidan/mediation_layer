// Cpp libraries
#include <ros/ros.h>

// Service type (for toggling shield)
#include "std_msgs/Empty.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "land_quads_example");
	ros::NodeHandle node("~");

	// message for requesting to land
	std_msgs::Empty msg;

	// Create publisher
	ros::Publisher land_pub = node.advertise<std_msgs::Empty>("/mediation_layer/land_quads", 1000);

	// Rate at which the publisher will run
    ros::Rate loop_rate(1);

	while(ros::ok()) {

		// Request to land all quads
	    land_pub.publish(msg);
	    ROS_INFO("[land_quads] Request to land quad...");
		
		// ROS loop that starts callbacks/publishers
		ros::spinOnce();

		// SLeep
	    loop_rate.sleep();
	}



	return 0;
}