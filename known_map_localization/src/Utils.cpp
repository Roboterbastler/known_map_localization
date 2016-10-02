/*
 * Utils.cpp
 *
 *  Created on: 02.10.2016
 *      Author: jacob
 */

#include <Utils.h>

#include <math.h>

namespace kml {

double degToRad(double degrees) {
	return degrees * M_PI / 180.;
}

double radToDeg(double radians) {
	return radians * 180. / M_PI;
}

double distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
	return sqrt(pow(p1.x - p2.x, 2.) + pow(p1.y - p2.y, 2.));
}

double median(std::vector<double> &values) {
	std::sort(values.begin(), values.end());
	size_t size = values.size();

	if (size % 2 == 0) {
		return (values.at(size / 2 - 1) + values.at(size / 2)) / 2.;
	} else {
		return values.at(values.size() / 2);
	}
}

}


