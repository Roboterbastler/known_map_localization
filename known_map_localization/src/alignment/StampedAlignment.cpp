/*
 * StampedAlignment.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <alignment/StampedAlignment.h>

namespace known_map_localization {
namespace alignment {

tf::StampedTransform StampedAlignment::toTfStampedTransform() const {
	return tf::StampedTransform(toTfTransform(), stamp, from, to);
}

StampedAlignment StampedAlignment::getIdentity() {
	StampedAlignment a;
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
