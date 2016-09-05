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

class VisualizationSlamMapPublisher;
typedef boost::shared_ptr<VisualizationSlamMapPublisher> VisualizationSlamMapPublisherPtr;
typedef boost::shared_ptr<VisualizationSlamMapPublisher const> VisualizationSlamMapPublisherConstPtr;

/**
 * # Visualization SLAM Map
 *
 * ## Published Topics
 * - **visualization_slam_map**: The corrected SLAM map
 */
class VisualizationSlamMapPublisher {
public:
	static VisualizationSlamMapPublisherPtr instance();

	/**
	 * Publishes the preprocessed SMAL map for visualization purposes.
	 * @param slamMap The SLAM map
	 */
	void publishSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap) const;

protected:
	VisualizationSlamMapPublisher();

private:
	static VisualizationSlamMapPublisherPtr _instance;

	/// Publishes the visualization SLAM map
	ros::Publisher slamMapPublisher;
};

} /* namespace visualization */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_VISUALIZATION_VISUALIZATIONSLAMMAPPUBLISHER_H_ */
