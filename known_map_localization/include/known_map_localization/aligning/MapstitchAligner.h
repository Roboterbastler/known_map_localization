/*
 * MapstitchAligner.h
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPSTITCHALIGNER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPSTITCHALIGNER_H_

#include <aligning/Aligner.h>

namespace known_map_localization {
namespace aligning {

class MapstitchAligner : public Aligner {
public:
	alignment::HypothesesVect align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap);

protected:
	static tf::Transform getOriginTransform(nav_msgs::OccupancyGridConstPtr map);
};

} /* namespace aligning */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPSTITCHALIGNER_H_ */
