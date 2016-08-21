/*
 * MapmergeAligner.h
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPMERGEALIGNER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPMERGEALIGNER_H_

#include <mapmerge/grid_map.h>

#include <aligning/Aligner.h>

namespace known_map_localization {
namespace aligning {

class MapmergeAligner: public Aligner {
public:
	MapmergeAligner();

	alignment::HypothesesVect align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap);
protected:
	/**
	 * Copies the map to a grid_map, assuming, that the target gridMap already has the needed size.
	 * @param occGrid The source map
	 * @param gridMap The target map
	 * @return True if map contains at least on occupied cell.
	 */
	static bool copyOccupancyGridToGridMap(nav_msgs::OccupancyGridConstPtr occGrid, mapmerge::grid_map &gridMap);

	/// This flag determines whether the randomized Hough transform should be used
	bool useRandomizedHoughTransform;

	/// This flag determines whether the robust version of the algorithm should be used
	bool useRobust;

	/// The number of hypotheses to be computed
	int numberOfHypotheses;
};

} /* namespace aligning */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNING_MAPMERGEALIGNER_H_ */
