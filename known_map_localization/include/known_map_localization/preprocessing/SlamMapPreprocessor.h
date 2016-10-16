/*
 * SlamMapPreprocessor.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_SLAMMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_SLAMMAPPREPROCESSOR_H_

#include "MapPreprocessor.h"

#include <SlamScaleManager.h>
#include <known_map_server/KnownMapServer.h>

namespace kml {

class SlamMapPreprocessor: public MapPreprocessor {
public:
	SlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer);

protected:
	bool scaleMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);

protected:
	SlamScaleManagerConstPtr pSlamScaleManager_;
	KnownMapServerConstPtr pKnownMapServer_;
};

typedef boost::shared_ptr<SlamMapPreprocessor> SlamMapPreprocessorPtr;
typedef boost::shared_ptr<SlamMapPreprocessor const> SlamMapPreprocessorConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_SLAMMAPPREPROCESSOR_H_ */
