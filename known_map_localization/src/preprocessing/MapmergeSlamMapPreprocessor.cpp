/*
 * MapmergeSlamMapPreprocessor.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <SlamScaleManager.h>
#include <preprocessing/MapmergeSlamMapPreprocessor.h>

#define KNOWN_MAP_RESOLUTION_UNAVAILABLE -1

namespace known_map_localization {
namespace preprocessing {

MapmergeSlamMapPreprocessor::MapmergeSlamMapPreprocessor() : knownMapResolution(KNOWN_MAP_RESOLUTION_UNAVAILABLE) {
	ros::NodeHandle nh("~");

	// subscribe to known map topic to get known map resolution
	knownMapSubscriber = nh.subscribe("visualization_known_map", 1, &MapmergeSlamMapPreprocessor::receiveKnownMap, this);
}

bool MapmergeSlamMapPreprocessor::processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData) {
	ROS_INFO("Preprocessing of SLAM map...");

	assert(img.rows == mapMetaData.height);
	assert(img.cols == mapMetaData.width);

	// scale SLAM map
	if(knownMapResolution == KNOWN_MAP_RESOLUTION_UNAVAILABLE) {
		ROS_WARN("Known map resolution unavailable, SLAM map preprocessing cancelled.");
		return false;
	}

	float realSlamResolution = mapMetaData.resolution * slam_scale_manager::SlamScaleManager::instance()->getSlamScale();
	float imgScaleFactor = realSlamResolution / knownMapResolution;
	cv::resize(img, img, cv::Size(), imgScaleFactor, imgScaleFactor, CV_INTER_NN);
	mapMetaData.resolution = realSlamResolution / imgScaleFactor;
	mapMetaData.origin = slam_scale_manager::SlamScaleManager::instance()->convertPoseMsg(mapMetaData.origin);
	mapMetaData.height = img.rows;
	mapMetaData.width = img.cols;

	// remove small particles (morphological close)
//	unsigned int kernelSize = std::ceil(0.1 / mapMetaData.resolution);
//	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*kernelSize+1, 2*kernelSize+1));
//	cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

	// crop image to occupied region
	crop(img, mapMetaData);

	return true;
}

void MapmergeSlamMapPreprocessor::receiveKnownMap(nav_msgs::OccupancyGridConstPtr knownMap) {
	knownMapResolution = knownMap->info.resolution;
}

} /* namespace preprocessing */
} /* namespace known_map_localization */
