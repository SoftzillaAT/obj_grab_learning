#include "obj_grab_learning_node.h"
#include <ros/ros.h>

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "obj_grab_learning");
	if(!ros::isInitialized())
		return 1;
	ros::NodeHandle n;
	ROS_INFO("OBJECT GRAB LEARNING STARTET!!");
	ros::spin();	
	return 0;
}
