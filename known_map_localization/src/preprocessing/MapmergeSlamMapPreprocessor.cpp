/*
 * MapmergeSlamMapPreprocessor.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <SlamScaleManager.h>
#include <preprocessing/MapmergeSlamMapPreprocessor.h>
#include <Exception.h>

#define KNOWN_MAP_RESOLUTION_UNAVAILABLE -1

namespace kml {

MapmergeSlamMapPreprocessor::MapmergeSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) :
		SlamMapPreprocessor(pSlamScaleManager, pKnownMapServer) {
}

bool MapmergeSlamMapPreprocessor::processMap(cv::Mat &img,
		nav_msgs::MapMetaData &mapMetaData) {
	ROS_INFO("Preprocessing of SLAM map...");

	ROS_ASSERT(img.rows == mapMetaData.height);
	ROS_ASSERT(img.cols == mapMetaData.width);

	// transform to correct SLAM map scale
	if(!scaleMap(img, mapMetaData)) {
		return false;
	}

	// crop image to occupied region
	crop(img, mapMetaData);

	return true;
}

} /* namespace kml */
