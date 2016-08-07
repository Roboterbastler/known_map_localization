/*
 * MapmergeSlamMapPreprocessor.h
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGESLAMMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGESLAMMAPPREPROCESSOR_H_

#include <preprocessing/SlamMapPreprocessor.h>

namespace known_map_localization {
namespace preprocessing {

class MapmergeSlamMapPreprocessor: public SlamMapPreprocessor {
public:
	MapmergeSlamMapPreprocessor();
protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);

	/// The a priori known scale of the SLAM map
	float scale;
};

} /* namespace preprocessing */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGESLAMMAPPREPROCESSOR_H_ */
