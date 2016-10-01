/*
 * CsMergeAligner.cpp
 *
 *  Created on: 30.09.2016
 *      Author: jacob
 */

#include <aligning/CsMergeAligner.h>

#include <cs_merge_msgs/getTransform.h>

namespace known_map_localization {
namespace aligning {

using namespace alignment;

CsMergeAligner::CsMergeAligner() : mMethod_(determineMethod()) {
	ros::NodeHandle nh("~");

	switch(mMethod_) {
	case CS_ICP_GRADIENT:
		mAligningClient_ = nh.serviceClient("");
		break;
	default:
		ROS_FATAL("CsMergeAligner: Invalid merge method!");
		ros::shutdown();
		return;
	}
}

alignment::HypothesesVect align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap) {
	ros::WallTime start = ros::WallTime::now();

	HypothesesVect resultHypotheses;

	// TODO

	ros::WallDuration duration = ros::WallTime::now() - start;
	ROS_INFO("Successfully completed aligning in %f sec", duration.toSec());

	return resultHypotheses;
}

CsMergeMethod CsMergeAligner::determineMethod() const {
	ros::NodeHandle nh("~");
	std::string invalid = "not specified";
	std::string mode = nh.param("cs_merge_method", invalid);
	std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);

	if (mode == "ICP_GRADIENT") {
		return CS_ICP_GRADIENT;
	}
	if (mode == "ICP_SVD") {
		return CS_ICP_SVD;
	}
	if (mode == "HOUGH_CCR") {
		return CS_HOUGH_CCR;
	}
	if (mode == "HOUGH_CORNER") {
		return CS_HOUGH_CORNER;
	}
	return CS_INVALID;
}

} /* namespace aligning */
} /* namespace known_map_localization */
