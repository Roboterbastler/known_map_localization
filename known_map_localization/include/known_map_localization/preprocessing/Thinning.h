/*
 * Thinning.h
 *
 *  Created on: 24.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_THINNING_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_THINNING_H_

/**
 * # Thinning algorithms
 *
 * Implementations of two thinning algorithms by Nashruddin Amin.
 *
 * Code from:
 * http://opencv-code.com/quick-tips/implementation-of-guo-hall-thinning-algorithm/
 * http://opencv-code.com/quick-tips/implementation-of-thinning-algorithm-in-opencv/
 */

namespace kml {

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
void thinningGuoHall(cv::Mat& im);

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
void thinningZhangSuen(cv::Mat& im);

}


#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_PREPROCESSING_THINNING_H_ */
