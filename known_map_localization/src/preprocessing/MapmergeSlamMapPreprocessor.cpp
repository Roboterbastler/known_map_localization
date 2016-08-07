/*
 * MapmergeSlamMapPreprocessor.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <preprocessing/MapmergeSlamMapPreprocessor.h>

namespace known_map_localization {
namespace preprocessing {

MapmergeSlamMapPreprocessor::MapmergeSlamMapPreprocessor() {
	ros::NodeHandle nh("~");
	if(!nh.getParam("slam_map_scale", scale)) {
		ROS_FATAL("SLAM map scale parameter not given!");
		ros::shutdown();
		return;
	}
}

bool MapmergeSlamMapPreprocessor::processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData) {
	// TODO: scale SLAM map
	return true;
}

} /* namespace preprocessing */
} /* namespace known_map_localization */
