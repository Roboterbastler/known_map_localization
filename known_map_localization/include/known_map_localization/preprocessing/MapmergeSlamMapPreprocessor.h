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
	MapmergeSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer);

protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_MAPMERGESLAMMAPPREPROCESSOR_H_ */
