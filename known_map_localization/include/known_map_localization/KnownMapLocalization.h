/*
 * KnownMapLocalization.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_

#include <ros/ros.h>

#include <AlgorithmSelector.h>
#include <known_map_server/KnownMapServer.h>
#include <filter/Filter.h>
#include <base_link/BaseLinkPublisher.h>
#include <visualization/VisualizationSlamMapPublisher.h>
#include <logging/DataLogger.h>

namespace known_map_localization {

/**
 * # Known Map Localization
 * Central control unit of the known map localization system.
 *
 * ## Published Topics
 *
 * ## Subscribed Topics
 * - **slam_map**: The SLAM map coming from the ORB_SLAM module
 *
 * ## Parameters
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
	/// selects the algorithms according to the method chosen
	AlgorithmSelectorPtr algorithmSelector;

	/// publishes the known map
	known_map_server::KnownMapServerConstPtr knownMapServer;

	/// filters the computed alignments
	filter::FilterPtr filter;

	/// publishes the base link based on the filtered alignment
	base_link::BaseLinkPublisherConstPtr baseLinkPublisher;

	/// Re-publishes a scaled version of the SLAM map for visualization purposes.
	visualization::VisualizationSlamMapPublisherConstPtr visualizationSlamMapPublisher;

	/// Subscribes to the map topic released by the SLAM package
	ros::Subscriber slamMapSubscriber;

	/// The aligning algorithm
	aligning::AlignerPtr aligner;

	/// The known map preprocessor
	preprocessing::KnownMapPreprocessorPtr knownMapPreprocessor;

	/// The SLAM map preprocessor
	preprocessing::SlamMapPreprocessorPtr slamMapPreprocessor;

	/// Data logger for later analysis
	logging::DataLogger dataLogger;
};

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_ */
