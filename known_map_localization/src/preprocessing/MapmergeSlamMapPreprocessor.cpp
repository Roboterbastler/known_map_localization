/*
 * MapmergeSlamMapPreprocessor.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <preprocessing/MapmergeSlamMapPreprocessor.h>

#define KNOWN_MAP_RESOLUTION_UNAVAILABLE -1

namespace known_map_localization {
namespace preprocessing {

MapmergeSlamMapPreprocessor::MapmergeSlamMapPreprocessor() : knownMapResolution(KNOWN_MAP_RESOLUTION_UNAVAILABLE) {
	ros::NodeHandle nh("~");
	if(!nh.getParam("slam_map_scale", scale)) {
		ROS_FATAL("SLAM map scale parameter not given!");
		ros::shutdown();
		return;
	}

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

	ROS_INFO("- Unprocessed SLAM map dimensions: x = %d y = %d", img.cols, img.rows);

	float realSlamResolution = mapMetaData.resolution / scale;
	float imgScaleFactor = realSlamResolution / knownMapResolution;
	cv::resize(img, img, cv::Size(), imgScaleFactor, imgScaleFactor, CV_INTER_NN);
	mapMetaData.resolution = realSlamResolution / imgScaleFactor;
	mapMetaData.height = img.rows;
	mapMetaData.width = img.cols;

	// restore map
	//cv::threshold(img, img, )

	ROS_INFO("- Real SLAM resolution: %f", realSlamResolution);
	ROS_INFO("- SLAM scale factor: %f", imgScaleFactor);
	ROS_INFO("- Known map resolution: %f", knownMapResolution);
	ROS_INFO("- Preprocessed SLAM map resolution: %f", mapMetaData.resolution);
	ROS_INFO("- Preprocessed SLAM map dimensions: x = %d y = %d", img.cols, img.rows);

	return true;
}

void MapmergeSlamMapPreprocessor::receiveKnownMap(nav_msgs::OccupancyGridConstPtr knownMap) {
	knownMapResolution = knownMap->info.resolution;
}

} /* namespace preprocessing */
} /* namespace known_map_localization */
