/*
 * scale_investigator_node.cpp
 *
 *  Created on: 01.08.2016
 *      Author: jacob
 */

#include <ros/ros.h>
#include "scale_investigator/ScaleInvestigator.h"

int main(int argc, char **argv) {
	// Initialize the ROS system
	ros::init(argc, argv, "scale_investigator");

	known_map_localization::ScaleInvestigator si;

	si.spin();
}
