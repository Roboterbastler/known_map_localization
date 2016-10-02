/*
 * CsMergeAligner.h
 *
 *  Created on: 30.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <cs_merge_msgs/getTransform.h>

#include <aligning/Aligner.h>

namespace kml {

/**
 * # cs_merge Aligner
 * This aligning algorithm uses the cs_merge package to align two maps.
 */
class CsMergeAligner: public Aligner {
public:
	CsMergeAligner(std::string serviceName);
	virtual ~CsMergeAligner() = 0;

	/**
	 * Aligns the two maps and returns an alignment.
	 * @param knownMap The known map
	 * @param slamMap The SLAM map
	 * @return A vector of hypotheses, with some aligning algorithms
	 * it just contains one hypothesis (the alignment)
	 */
	HypothesesVect align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap);

protected:
	/// Service client for the aligning service call
	ros::ServiceClient mAligningClient_;
};

/**
 * Converts a transform from cs_merge's data type to tf Transform.
 * @param transform The cs_merge transform
 * @return The tf transform
 */
tf::Transform transformCsToTf(const cs_merge_msgs::transform &transform);

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_ */
