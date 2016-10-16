/*
 * MapstitchSlamMapPreprocessor.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <preprocessing/MapstitchSlamMapPreprocessor.h>

namespace kml {

MapstitchSlamMapPreprocessor::MapstitchSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) :
		SlamMapPreprocessor(pSlamScaleManager, pKnownMapServer) {

}

bool MapstitchSlamMapPreprocessor::processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData) {

	// get correct SLAM map scale
	if(!scaleMap(img, mapMetaData)) {
		return false;
	}

	// close small holes...
	morphologicalOpen(img);

	// ... and remove remaining noise
	morphologicalClose(img);

	// crop to region of interest
	crop(img, mapMetaData);

	return true;
}

} /* namespace kml */
