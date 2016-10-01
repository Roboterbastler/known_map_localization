/*
 * MapstitchSlamMapPreprocessor.h
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPSTITCHSLAMMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPSTITCHSLAMMAPPREPROCESSOR_H_

#include <preprocessing/SlamMapPreprocessor.h>

namespace kml {

class MapstitchSlamMapPreprocessor: public SlamMapPreprocessor {
protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPSTITCHSLAMMAPPREPROCESSOR_H_ */
