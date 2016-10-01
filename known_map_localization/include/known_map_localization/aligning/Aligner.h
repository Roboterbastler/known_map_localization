/*
 * Aligner.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_ALIGNER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_ALIGNER_H_

#include <nav_msgs/OccupancyGrid.h>

#include "alignment/Hypothesis.h"

namespace kml {

/**
 * # Aligner
 *
 * Base class for aligning algorithms, that take two maps as occupancy grid maps
 * and return one or multiple hypotheses about how the should be aligned.
 */
class Aligner {
public:
	virtual ~Aligner();

	/**
	 * Aligns the two maps and returns an alignment.
	 * @param knownMap The known map
	 * @param slamMap The SLAM map
	 * @return A vector of hypotheses, with some aligning algorithms
	 * it just contains one hypothesis (the alignment)
	 */
	virtual HypothesesVect align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap) = 0;
};

typedef boost::shared_ptr<Aligner> AlignerPtr;
typedef boost::shared_ptr<Aligner const> AlignerConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_ALIGNER_H_ */
