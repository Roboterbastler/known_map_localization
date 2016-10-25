/*
 * IcpKnownMapPreprocessor.h
 *
 *  Created on: 25.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_ICPKNOWNMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_ICPKNOWNMAPPREPROCESSOR_H_

#include <preprocessing/KnownMapPreprocessor.h>

namespace kml {

class IcpKnownMapPreprocessor: public KnownMapPreprocessor {
protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_ICPKNOWNMAPPREPROCESSOR_H_ */
