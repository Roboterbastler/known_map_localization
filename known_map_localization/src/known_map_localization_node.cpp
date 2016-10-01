/*
 * known_map_localization_node.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <ros/ros.h>
#include <KnownMapLocalization.h>

int main(int argc, char **argv) {
	// Initialize the ROS system
	ros::init(argc, argv, "known_map_localization");

	ROS_INFO_STREAM("---------- Known Map Localization initialization started...");

	kml::KnownMapLocalization system;

	ROS_INFO_STREAM("---------- Known Map Localization initialization finished.");

	ros::spin();
}
