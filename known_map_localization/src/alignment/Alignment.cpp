/*
 * Alignment.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include "Alignment.h"

namespace known_map_localization {
namespace alignment {

tf::Transform Alignment::toTfTransform() const {
	tf::Quaternion q;
	q.setRPY(0, 0, theta);
	return tf::Transform(q, tf::Vector3(x, y, 0));
}

} /* namespace alignment */
} /* namespace known_map_localization */
