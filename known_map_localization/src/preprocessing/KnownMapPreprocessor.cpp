/*
 * KnownMapPreprocessor.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <preprocessing/KnownMapPreprocessor.h>

namespace kml {

KnownMapPreprocessor::KnownMapPreprocessor() :
		MapPreprocessor("processed_known_map_img", "known_preprocessing_enabled") {
	ROS_INFO("Known map preprocessor initialization...");
}

} /* namespace kml */
