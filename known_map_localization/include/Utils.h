/*
 * Utils.h
 *
 *  Created on: 06.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_UTILS_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_UTILS_H_

#include <math.h>

namespace known_map_localization {

/**
 * Conversion from radians to degree.
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
inline double degToRad(double degrees) {
	return degrees * M_PI / 180.;
}

/**
 * Conversion from radians to degree.
 * @param radians Angle in radians
 * @return Angle in degrees
 */
inline double radToDeg(double radians) {
	return radians * 180. / M_PI;
}

/**
 * Gets the yaw from a quaternion.
 * @param quaternion The quaternion
 * @return The yaw in radians
 */
inline double quaternionToYawRad(const tf::Quaternion &quaternion) {
	double roll, pitch, yaw;
	tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
	return yaw;
}

/**
 * Gets the yaw from a quaternion.
 * @param quaternion The quaternion
 * @return The yaw in degrees
 */
inline double quaternionToYawDegree(const tf::Quaternion &quaternion) {
	return radToDeg(quaternionToYawRad(quaternion));
}

}

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_UTILS_H_ */
