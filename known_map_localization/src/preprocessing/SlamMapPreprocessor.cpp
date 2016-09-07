/*
 * SlamMapPreprocessor.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <preprocessing/SlamMapPreprocessor.h>
#include <AlgorithmSelector.h>
#include <preprocessing/MapstitchSlamMapPreprocessor.h>
#include <preprocessing/MapmergeSlamMapPreprocessor.h>
#include <Exception.h>

namespace known_map_localization {
namespace preprocessing {

SlamMapPreprocessorPtr SlamMapPreprocessor::_instance;

SlamMapPreprocessor::SlamMapPreprocessor() :
		MapPreprocessor("processed_slam_map_img", "slam_preprocessing_enabled") {
	ROS_INFO("SLAM map preprocessor initialization...");
}

SlamMapPreprocessorPtr SlamMapPreprocessor::instance() {
	if(!_instance) {
		switch(AlgorithmSelector::determineAlgorithm()) {
		case MAPSTITCH:
				_instance = SlamMapPreprocessorPtr(new MapstitchSlamMapPreprocessor());
				break;
			case MAPMERGE:
				_instance = SlamMapPreprocessorPtr(new MapmergeSlamMapPreprocessor());
				break;
			case CSMERGE:
			default:
				throw IllegalAlgorithm("Invalid algorithm");
		}
	}
	return _instance;
}

} /* namespace preprocessing */
} /* namespace known_map_localization */
