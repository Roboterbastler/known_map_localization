/*
 * IcpKnownMapPreprocessor.cpp
 *
 *  Created on: 25.10.2016
 *      Author: jacob
 */

#include <preprocessing/IcpKnownMapPreprocessor.h>

namespace kml {

bool IcpKnownMapPreprocessor::processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData) {
	// edge detection using Canny detector
	cv::Mat edges;
	cv::Canny(img, edges, 50, 150, 3);
	edges.convertTo(img, CV_8U);

	// invert
	img = cv::Scalar::all(255) - img;

	// restore to correct free value
	restoreMap(img);

	return true;
}
} /* namespace kml */
