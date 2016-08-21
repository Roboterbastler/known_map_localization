/*
 * VisualizationSlamMapPublisher.h
 *
 *  Created on: 06.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_VISUALIZATION_VISUALIZATIONSLAMMAPPUBLISHER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_VISUALIZATION_VISUALIZATIONSLAMMAPPUBLISHER_H_

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <filter/Filter.h>

namespace known_map_localization {
namespace visualization {

/**
 * # Visualization SLAM Map
 *
 * ## Published Topics
 * - **visualization_slam_map**: The corrected SLAM map
 */
class VisualizationSlamMapPublisher {
public:
	VisualizationSlamMapPublisher(filter::FilterConstPtr filter);

	/**
	 * Publishes the preprocessed SMAL map for visualization purposes.
	 * @param slamMap The SLAM map
	 */
	void publishSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap) const;

private:

	/// Publishes the visualization SLAM map
	ros::Publisher slamMapPublisher;

	/// The filter to get the alignment from
	filter::FilterConstPtr filter;
};

typedef boost::shared_ptr<VisualizationSlamMapPublisher> VisualizationSlamMapPublisherPtr;
typedef boost::shared_ptr<VisualizationSlamMapPublisher const> VisualizationSlamMapPublisherConstPtr;
} /* namespace visualization */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_VISUALIZATION_VISUALIZATIONSLAMMAPPUBLISHER_H_ */
