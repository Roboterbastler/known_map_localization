/*
 * MapstitchAligner.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <aligning/MapstitchAligner.h>

namespace known_map_localization {
namespace aligning {

alignment::StampedAlignment MapstitchAligner::align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap) {
	// TODO: implement mapstitch call
	return alignment::StampedAlignment::getIdentity();
}

} /* namespace aligning */
} /* namespace known_map_localization */
