/*
 * VisualizationSlamMapPublisher.cpp
 *
 *  Created on: 06.08.2016
 *      Author: jacob
 */

#include <visualization/VisualizationSlamMapPublisher.h>
#include <alignment/Alignment.h>
#include <Exception.h>

namespace known_map_localization {
namespace visualization {

VisualizationSlamMapPublisher::VisualizationSlamMapPublisher(filter::FilterConstPtr filter) : filter(filter) {
	ros::NodeHandle nh("~");
	slamMapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("visualization_slam_map", 10);
}

void VisualizationSlamMapPublisher::publishSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap) const {
	nav_msgs::OccupancyGridPtr correctedSlamMap(new nav_msgs::OccupancyGrid(*slamMap));
	alignment::Alignment alignment;
	try {
		alignment = filter->getAlignment();
	} catch(AlignmentNotAvailable &e) {
		return;
	}

	correctedSlamMap->info.resolution *= alignment.scale;
	correctedSlamMap->info.origin.position.x *= alignment.scale;
	correctedSlamMap->info.origin.position.y *= alignment.scale;

	slamMapPublisher.publish(correctedSlamMap);
}

} /* namespace visualization */
} /* namespace known_map_localization */
