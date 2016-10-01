/*
 * SlamMapPreprocessor.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <preprocessing/SlamMapPreprocessor.h>

namespace kml {

SlamMapPreprocessor::SlamMapPreprocessor() :
		MapPreprocessor("processed_slam_map_img", "slam_preprocessing_enabled") {
	ROS_INFO("SLAM map preprocessor initialization...");
}

} /* namespace kml */
