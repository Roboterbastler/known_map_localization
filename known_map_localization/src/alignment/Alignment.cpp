/*
 * Alignment.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <alignment/Alignment.h>

namespace kml {

Alignment::Alignment() :
		scale(1), theta(0), x(0), y(0), from(""), to("") {
}

tf::Transform Alignment::toTfTransform() const {
	tf::Quaternion q;
	q.setRPY(0, 0, theta);
	return tf::Transform(q, tf::Vector3(x, y, 0));
}

Alignment Alignment::getIdentity() {
	return Alignment();
}

} /* namespace kml */
