/*
 * KnownMapLocalization.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace known_map_localization {

/**
 * # Known Map Localization
 * Central control unit of the known map localization system.
 *
 * ## Published Topics
 *
 * ## Subscribed Topics
 * - **slam_map**: The SLAM map coming from the ORB_SLAM module
 */
class KnownMapLocalization {
public:
	KnownMapLocalization();

private:
	/**
	 * This callback function is used by the subscriber to the occupancy grid map coming in from ORB_SLAM.
	 * @param slamMap The nav_msgs/OccupancyGrid message
	 */
	void receiveSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap);

private:
	/// Subscribes to the map topic released by the SLAM package
	ros::Subscriber slamMapSubscriber;
};

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_ */
