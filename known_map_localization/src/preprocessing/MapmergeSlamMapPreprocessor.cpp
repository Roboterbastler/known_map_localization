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

MapmergeSlamMapPreprocessor::MapmergeSlamMapPreprocessor(
		SlamScaleManagerConstPtr pSlamScaleManager) :
		mKnownMapResolution_(KNOWN_MAP_RESOLUTION_UNAVAILABLE), pSlamScaleManager_(
				pSlamScaleManager) {
	ROS_ASSERT(pSlamScaleManager_);

	ros::NodeHandle nh("~");

	// subscribe to known map topic to get known map resolution
	mKnownMapSubscriber_ = nh.subscribe("visualization_known_map", 1,
			&MapmergeSlamMapPreprocessor::receiveKnownMap, this);
}

bool MapmergeSlamMapPreprocessor::processMap(cv::Mat &img,
		nav_msgs::MapMetaData &mapMetaData) {
	ROS_INFO("Preprocessing of SLAM map...");

	ROS_ASSERT(img.rows == mapMetaData.height);
	ROS_ASSERT(img.cols == mapMetaData.width);

	// scale SLAM map
	if (mKnownMapResolution_ == KNOWN_MAP_RESOLUTION_UNAVAILABLE) {
		ROS_WARN(
				"Known map resolution unavailable, SLAM map preprocessing cancelled.");
		return false;
	}

	try {
		float realSlamResolution = mapMetaData.resolution
				* pSlamScaleManager_->getSlamScale();
		float imgScaleFactor = realSlamResolution / mKnownMapResolution_;
		cv::resize(img, img, cv::Size(), imgScaleFactor, imgScaleFactor,
				CV_INTER_NN);
		mapMetaData.resolution = realSlamResolution / imgScaleFactor;
		mapMetaData.origin = pSlamScaleManager_->convertPoseMsg(
				mapMetaData.origin);
		mapMetaData.height = img.rows;
		mapMetaData.width = img.cols;
	} catch (ScaleNotAvailable &e) {
		ROS_WARN("SLAM map preprocessing aborted: %s", e.what());
		return false;
	}

	// remove small particles (morphological close)
//	unsigned int kernelSize = std::ceil(0.1 / mapMetaData.resolution);
//	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*kernelSize+1, 2*kernelSize+1));
//	cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

// crop image to occupied region
	crop(img, mapMetaData);

	return true;
}

void MapmergeSlamMapPreprocessor::receiveKnownMap(
		nav_msgs::OccupancyGridConstPtr knownMap) {
	mKnownMapResolution_ = knownMap->info.resolution;
}

} /* namespace kml */
