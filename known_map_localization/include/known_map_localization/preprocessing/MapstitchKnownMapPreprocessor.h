/*
 * MapstitchKnownMapPreprocessor.h
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPSTITCHKNOWNMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPSTITCHKNOWNMAPPREPROCESSOR_H_

#include <preprocessing/KnownMapPreprocessor.h>

namespace kml {

class MapstitchKnownMapPreprocessor: public KnownMapPreprocessor {
protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPSTITCHKNOWNMAPPREPROCESSOR_H_ */
