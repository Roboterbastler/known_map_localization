/*
 * MapPreprocessor.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <preprocessing/MapPreprocessor.h>

#include <cv_bridge/cv_bridge.h>
#include <mapstitch/utils.h>

namespace known_map_localization {
namespace preprocessing {

MapPreprocessor::MapPreprocessor(std::string topicName, std::string paramName) :
		enabled(false) {
	if (ros::isInitialized()) {
		ros::NodeHandle nh("~");

		preprocessedMapPublisher = nh.advertise<sensor_msgs::Image>(topicName,
				1, true);
		nh.getParam(paramName, enabled);
	}
}

MapPreprocessor::~MapPreprocessor() {
}

bool MapPreprocessor::process(nav_msgs::OccupancyGridPtr map) {
	assert(map);
	bool success = true;

	// convert to matrix
	cv::Mat img = matFromOccupancyGrid(map);

	if (enabled) {
		success = processMap(img, map->info);
		overwriteMapContent(map, img);
	}

	publishResult(img);
	return success;
}

cv::Mat MapPreprocessor::matFromOccupancyGrid(
		nav_msgs::OccupancyGridConstPtr map) {
	return occupancyGridToCvMat(map.get());
}

nav_msgs::OccupancyGridPtr MapPreprocessor::occupancyGridFromMat(
		const cv::Mat &img) {
	return boost::make_shared<nav_msgs::OccupancyGrid>(cvMatToOccupancyGrid(&img));
}

void MapPreprocessor::overwriteMapContent(nav_msgs::OccupancyGridPtr map,
		const cv::Mat &content) {
	nav_msgs::OccupancyGridConstPtr preprocessedMap = occupancyGridFromMat(content);
	map->data = preprocessedMap->data;
	map->info.width = preprocessedMap->info.width;
	map->info.height = preprocessedMap->info.height;
}

void MapPreprocessor::publishResult(const cv::Mat &img) {
	if(ros::isInitialized()) {
		cv_bridge::CvImage intermediateResultImage;
		intermediateResultImage.encoding = sensor_msgs::image_encodings::MONO8;
		intermediateResultImage.image = img;
		intermediateResultImage.header.stamp = ros::Time::now();
		preprocessedMapPublisher.publish(intermediateResultImage.toImageMsg());
	}
}

} /* namespace preprocessing */
} /* namespace known_map_localization */
