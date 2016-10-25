/*
 * IcpSlamMapPreprocessor.cpp
 *
 *  Created on: 25.10.2016
 *      Author: jacob
 */

#include <preprocessing/IcpSlamMapPreprocessor.h>

namespace kml {

IcpSlamMapPreprocessor::IcpSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) :
		SlamMapPreprocessor(pSlamScaleManager, pKnownMapServer) {
}

bool IcpSlamMapPreprocessor::processMap(cv::Mat &img,
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

	morphologicalOpen(img);

	morphologicalSkeleton(img);

	return true;
}

} /* namespace kml */
