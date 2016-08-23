/*
 * MapmergeKnownMapPreprocessor.h
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGEKNOWNMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGEKNOWNMAPPREPROCESSOR_H_

#include <preprocessing/KnownMapPreprocessor.h>

namespace known_map_localization {
namespace preprocessing {

class MapmergeKnownMapPreprocessor: public KnownMapPreprocessor {
protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);
};

} /* namespace preprocessing */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGEKNOWNMAPPREPROCESSOR_H_ */
