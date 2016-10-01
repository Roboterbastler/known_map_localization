/*
 * CsMergeAligner.h
 *
 *  Created on: 30.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_

#include <aligning/Aligner.h>

namespace known_map_localization {
namespace aligning {

/// The available aligning methods of cs_merge
typedef enum {
	CS_INVALID = 0,
	CS_ICP_GRADIENT,
	CS_ICP_SVD,
	CS_HOUGH_CCR,
	CS_HOUGH_CORNER
} CsMergeMethod;

/**
 * # cs_merge Aligner
 * This aligning algorithm uses the cs_merge package to align two maps.
 */
class CsMergeAligner: public Aligner {
public:
	CsMergeAligner();

	/**
	 * Aligns the two maps using the cs_merge package and returns a vector containing the hypothesis.
	 * @param knownMap The known map
	 * @param slamMap The SLAM map
	 * @return A vector of hypotheses
	 */
	alignment::HypothesesVect align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap);

private:
	CsMergeMethod determineMethod() const;

private:
	ros::ServiceClient mAligningClient_;

	CsMergeMethod mMethod_;
};

} /* namespace aligning */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_ */
