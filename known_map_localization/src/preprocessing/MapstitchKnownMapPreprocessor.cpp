/*
 * MapstitchKnownMapPreprocessor.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <preprocessing/MapstitchKnownMapPreprocessor.h>

namespace kml {

bool MapstitchKnownMapPreprocessor::processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData) {

	morphologicalErode(img, 2);

	return true;
}

} /* namespace kml */
