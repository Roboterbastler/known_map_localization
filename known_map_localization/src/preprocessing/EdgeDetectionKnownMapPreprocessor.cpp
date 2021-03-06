/*
 * MapmergeKnownMapPreprocessor.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <preprocessing/EdgeDetectionKnownMapPreprocessor.h>

namespace kml {

bool EdgeDetectionKnownMapPreprocessor::processMap(cv::Mat &img, nav_msgs::MapMetaData &mapMetaData) {
	// edge detection using Canny detector
	cv::Mat edges;
	cv::Canny(img, edges, 50, 150, 3);
	edges.convertTo(img, CV_8U);

	// invert
	img = cv::Scalar::all(255) - img;

	// make edges thicker
	unsigned int kernelRadius = 1;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*kernelRadius+1, 2*kernelRadius+1));
	cv::erode(img, img, kernel);

	// restore to correct free value
	restoreMap(img);

	return true;
}

} /* namespace kml */
