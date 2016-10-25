/*
 * IcpSlamMapPreprocessor.h
 *
 *  Created on: 25.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_ICPSLAMMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_ICPSLAMMAPPREPROCESSOR_H_

#include <preprocessing/SlamMapPreprocessor.h>

namespace kml {

class IcpSlamMapPreprocessor: public SlamMapPreprocessor {
public:
	IcpSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer);

protected:
	virtual bool processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData);
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_ICPSLAMMAPPREPROCESSOR_H_ */
