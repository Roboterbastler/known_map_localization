/*
 * MapPreprocessor.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPPREPROCESSOR_H_

#include <string>

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace kml {

class MapPreprocessor {
public:
	MapPreprocessor(std::string topicName, std::string paramName);
	virtual ~MapPreprocessor();

	bool process(nav_msgs::OccupancyGridPtr map);

protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData) = 0;

	static void updateMapOrigin(nav_msgs::MapMetaData &map, cv::Point2i origin);
	static void crop(cv::Mat &img, nav_msgs::MapMetaData &map);
	static void restoreMap(cv::Mat &img);
	static void morphologicalOpen(cv::Mat &img, unsigned int kernelSize = 1);
	static void morphologicalClose(cv::Mat &img, unsigned int kernelSize = 1);
	static void morphologicalSkeleton(cv::Mat &img);
	static void morphologicalErode(cv::Mat &img, unsigned int kernelSize = 1);
	static void morphologicalDilate(cv::Mat &img, unsigned int kernelSize = 1);

private:
	static cv::Mat matFromOccupancyGrid(nav_msgs::OccupancyGridConstPtr map);
	static nav_msgs::OccupancyGridPtr occupancyGridFromMat(const cv::Mat &img);
	static void overwriteMapContent(nav_msgs::OccupancyGridPtr map, const cv::Mat &content);
	void publishResult(const cv::Mat &img);

	ros::Publisher mPreprocessedMapPublisher_;
	bool mEnabled_;
};

typedef boost::shared_ptr<MapPreprocessor> MapPreprocessorPtr;
typedef boost::shared_ptr<MapPreprocessor const> MapPreprocessorConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPPREPROCESSOR_H_ */
