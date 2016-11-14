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
	morphologicalOpen(img, 3);

	// ... and remove remaining noise
	//morphologicalClose(img);

	// crop to region of interest
	crop(img, mapMetaData);

	// fill unknown regions with white (to avoid additional corners)
	cv::threshold(img, img, 50, 254, cv::THRESH_BINARY);

	//morphologicalErode(img);

//	morphologicalClose(img);

	restoreMap(img);

	return true;
}

} /* namespace kml */
