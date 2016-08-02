/*
 * SlamMapPreprocessor.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include "SlamMapPreprocessor.h"

namespace known_map_localization {
namespace preprocessing {

SlamMapPreprocessor::SlamMapPreprocessor() :
		MapPreprocessor("processed_slam_map", "slam_preprocessing_enabled") {

}

} /* namespace preprocessing */
} /* namespace known_map_localization */
