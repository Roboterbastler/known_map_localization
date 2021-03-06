/*
 * MapstitchAligner.h
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPSTITCHALIGNER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPSTITCHALIGNER_H_

#include <aligning/Aligner.h>

namespace kml {

/**
 * # Mapstitch Aligner
 *
 * This aligning algorithm uses the mapstitch package to align two maps.
 * It only returns one hypothesis.
 *
 * ## Parameters
 * - **max_pairwise_distance**: The max. distance between features. See mapstitch documentation for more.
 */
class MapstitchAligner : public Aligner {
public:
	/**
	 * Aligns the two maps using the mapstitch package and returns one hypothesis.
	 * @param knownMap The known map
	 * @param slamMap The SLAM map
	 * @return A vector containing the hypothesis
	 */
	HypothesesVect align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap);
};

tf::Transform getOriginTransform(nav_msgs::OccupancyGridConstPtr map);

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPSTITCHALIGNER_H_ */
