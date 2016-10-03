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

#include <factory/KmlFactory.h>

namespace kml {

/**
 * # Known Map Localization
 * Central control unit of the known map localization system.
 *
 * ## Published Topics
 * - **visualization_slam_map**: The corrected SLAM map
 *
 * ## Subscribed Topics
 * - **slam_map**: The SLAM map coming from the ORB_SLAM module
 *
 * ## Parameter
 * - **algorithm**: Determines the selected strategy
 */
class KnownMapLocalization {
public:
	KnownMapLocalization();

private:
	/**
	 * Return factory object according to the selected strategy.
	 * @return Factory object
	 */
	KmlFactoryConstPtr selectStrategy() const;

	/**
	 * This callback function is used by the subscriber to the occupancy grid map coming in from ORB_SLAM.
	 * @param slamMap The nav_msgs/OccupancyGrid message
	 */
	void receiveSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap);

private:
	/// Subscribes to the map topic released by the SLAM package
	ros::Subscriber mSlamMapSubscriber_;

	/// Publishes the visualization SLAM map
	ros::Publisher mSlamMapPublisher_;

	/// Minimal duration between processing of SLAM maps
	ros::WallDuration mRate_;
	ros::WallTime mLastProcessing_;

private:
	DataLoggerPtr pDataLogger_;
	KnownMapPreprocessorPtr pKnownMapPreprocessor_;
	SlamMapPreprocessorPtr pSlamMapPreprocessor_;
	AlignerPtr pAligner_;
	KnownMapServerConstPtr pKnownMapServer_;
	GpsManagerConstPtr pGpsManager_;
	SlamScaleManagerPtr pSlamScaleManager_;
	FilterPtr pFilter_;
	BaseLinkPublisherConstPtr pBaseLinkPublisher_;
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_ */
