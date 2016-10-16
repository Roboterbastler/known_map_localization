/*
 * SlamMapPreprocessor.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <preprocessing/SlamMapPreprocessor.h>

#include <Exception.h>

namespace kml {

SlamMapPreprocessor::SlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) :
		MapPreprocessor("processed_slam_map_img", "slam_preprocessing_enabled"),
		pSlamScaleManager_(pSlamScaleManager),
		pKnownMapServer_(pKnownMapServer) {
	ROS_ASSERT(pSlamScaleManager_);
	ROS_ASSERT(pKnownMapServer_);

	ROS_INFO("SLAM map preprocessor initialization...");
}

bool SlamMapPreprocessor::scaleMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData) {
	try {
		float realSlamResolution = mapMetaData.resolution
				* pSlamScaleManager_->getSlamScale();
		float imgScaleFactor = realSlamResolution / pKnownMapServer_->getKnownMap()->info.resolution;
		cv::resize(img, img, cv::Size(), imgScaleFactor, imgScaleFactor,
				CV_INTER_NN);
		mapMetaData.resolution = realSlamResolution / imgScaleFactor;
		mapMetaData.origin = pSlamScaleManager_->convertPoseMsg(
				mapMetaData.origin);
		mapMetaData.height = img.rows;
		mapMetaData.width = img.cols;

		return true;
	} catch (ScaleNotAvailable &e) {
		ROS_INFO("SLAM map preprocessing aborted: %s", e.what());
		return false;
	}
}

} /* namespace kml */
