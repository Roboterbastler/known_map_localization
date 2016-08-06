/*
 * Alignment.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <alignment/Alignment.h>

namespace known_map_localization {
namespace alignment {

tf::Transform Alignment::toTfTransform() const {
	tf::Quaternion q;
	q.setRPY(0, 0, theta);
	return tf::Transform(q, tf::Vector3(x, y, 0));
}

Alignment Alignment::getIdentity() {
	Alignment a;
	a.from = "";
	a.to = "";
	a.scale = 1.;
	a.theta = 0.;
	a.x = 0.;
	a.y = 0.;
	return a;
}

} /* namespace alignment */
} /* namespace known_map_localization */
