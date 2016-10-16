/*
 * MapPreprocessor.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <opencv2/imgproc/imgproc.hpp>

#include <preprocessing/MapPreprocessor.h>

#include <cv_bridge/cv_bridge.h>
#include <mapstitch/utils.h>
#include <tf/transform_datatypes.h>

namespace kml {

MapPreprocessor::MapPreprocessor(std::string topicName, std::string paramName) :
		mEnabled_(false) {
	if (ros::isInitialized()) {
		ros::NodeHandle nh("~");

		mPreprocessedMapPublisher_ = nh.advertise<sensor_msgs::Image>(topicName,
				1, true);
		nh.getParam(paramName, mEnabled_);
	}
}

MapPreprocessor::~MapPreprocessor() {
}

bool MapPreprocessor::process(nav_msgs::OccupancyGridPtr map) {
	ROS_ASSERT(map);
	bool success = true;

	// convert to matrix
	cv::Mat img = matFromOccupancyGrid(map);

	if (mEnabled_) {
		success = processMap(img, map->info);
		overwriteMapContent(map, img);
	}

	if(success) {
		publishResult(img);
	} else {
		ROS_WARN("Map preprocessing failed.");
	}

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
		mPreprocessedMapPublisher_.publish(intermediateResultImage.toImageMsg());
	}
}

void MapPreprocessor::crop(cv::Mat &img, nav_msgs::MapMetaData &map) {
	cv::Mat points, mask;
	cv::threshold(img, mask, 1, 255, cv::THRESH_BINARY_INV);
	cv::findNonZero(mask, points);

	if(points.rows == 0) {
		ROS_DEBUG("Preprocessor: Map does not contain occupied cells. Skip cropping.");
		return;
	}

	cv::Rect boundingBox = cv::boundingRect(points);

	// crop
	img = img(boundingBox).clone();

	cv::Point2i offset(boundingBox.x, boundingBox.y);
	updateMapOrigin(map, offset);

	map.height = img.rows;
	map.width = img.cols;
}

void MapPreprocessor::updateMapOrigin(nav_msgs::MapMetaData &map, cv::Point2i origin) {
	tf::Pose offset(tf::Quaternion::getIdentity(), tf::Vector3(origin.x * map.resolution, origin.y * map.resolution, 0));

	tf::Transform oldOrigin;
	tf::poseMsgToTF(map.origin, oldOrigin);
	tf::Transform newOrigin = oldOrigin * offset;

	tf::poseTFToMsg(newOrigin, map.origin);
}

void MapPreprocessor::restoreMap(cv::Mat &img) {
	// truncate image to max value 254 (which means free)
	cv::threshold(img, img, 254, 0, cv::THRESH_TRUNC);
}

void MapPreprocessor::morphologicalOpen(cv::Mat &img, unsigned int kernelSize) {
	// close small holes (morphological open)
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*kernelSize+1, 2*kernelSize+1));
	cv::morphologyEx(img, img, cv::MORPH_OPEN, kernel);
}

void MapPreprocessor::morphologicalClose(cv::Mat &img, unsigned int kernelSize) {
	// remove small particles (morphological close)
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*kernelSize+1, 2*kernelSize+1));
	cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);
}

void MapPreprocessor::morphologicalSkeleton(cv::Mat &img) {
	// algorithm taken from http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/

	// invert
	cv::threshold(img, img, 0, 255, cv::THRESH_BINARY_INV);
	cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp;
	cv::Mat eroded;

	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	bool done;
	do
	{
	  cv::erode(img, eroded, element);
	  cv::dilate(eroded, temp, element); // temp = open(img)
	  cv::subtract(img, temp, temp);
	  cv::bitwise_or(skel, temp, skel);
	  eroded.copyTo(img);

	  done = (cv::countNonZero(img) == 0);
	} while (!done);

	// invert
	img = cv::Scalar::all(255) - skel;
}

void MapPreprocessor::morphologicalErode(cv::Mat &img, unsigned int kernelSize) {
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*kernelSize+1, 2*kernelSize+1));
	cv::morphologyEx(img, img, cv::MORPH_ERODE, kernel);
}

void MapPreprocessor::morphologicalDilate(cv::Mat &img, unsigned int kernelSize) {
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*kernelSize+1, 2*kernelSize+1));
	cv::morphologyEx(img, img, cv::MORPH_DILATE, kernel);
}

} /* namespace kml */
