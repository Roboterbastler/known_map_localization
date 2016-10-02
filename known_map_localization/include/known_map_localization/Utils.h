/*
 * Utils.h
 *
 *  Created on: 06.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_UTILS_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_UTILS_H_

#include <geometry_msgs/Point.h>

namespace kml {

/**
 * Conversion from radians to degree.
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
double degToRad(double degrees);

/**
 * Conversion from radians to degree.
 * @param radians Angle in radians
 * @return Angle in degrees
 */
double radToDeg(double radians);

/**
 * Computes the distance between two points (on the xy plane).
 * @param p1 The first point
 * @param p2 The second point
 * @return The distance
 */
double distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

/**
 * Computes the median of a vector of values.
 * @param values The values
 * @return The median
 */
double median(std::vector<double> &values);

}

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_UTILS_H_ */
