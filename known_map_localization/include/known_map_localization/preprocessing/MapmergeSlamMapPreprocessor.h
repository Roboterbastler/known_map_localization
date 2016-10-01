/*
 * MapmergeSlamMapPreprocessor.h
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGESLAMMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGESLAMMAPPREPROCESSOR_H_

#include <preprocessing/SlamMapPreprocessor.h>

#include <SlamScaleManager.h>

namespace kml {

class MapmergeSlamMapPreprocessor: public SlamMapPreprocessor {
public:
	MapmergeSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager);

protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);

private:
	/**
	 * Callback function used to receive known map meta data.
	 * @param knownMap The received known map
	 */
	void receiveKnownMap(nav_msgs::OccupancyGridConstPtr knownMap);

	/// The resolution of the known map used to rescale the SLAM map.
	float mKnownMapResolution_;

	/// Subscribes to the known map topic
	ros::Subscriber mKnownMapSubscriber_;

private:
	SlamScaleManagerConstPtr pSlamScaleManager_;
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGESLAMMAPPREPROCESSOR_H_ */
