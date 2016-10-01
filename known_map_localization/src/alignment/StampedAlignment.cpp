/*
 * StampedAlignment.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <alignment/StampedAlignment.h>

namespace kml {

StampedAlignment::StampedAlignment() :
		Alignment(), stamp(ros::Time::now()) {
}

StampedAlignment::StampedAlignment(const Alignment &a) :
		Alignment(a), stamp(ros::Time::now()) {
}

tf::StampedTransform StampedAlignment::toTfStampedTransform() const {
	return tf::StampedTransform(toTfTransform(), stamp, from, to);
}

StampedAlignment StampedAlignment::getIdentity() {
	return StampedAlignment(Alignment::getIdentity());
}

typedef std::vector<StampedAlignment> StampedHypothesesVect;

} /* namespace kml */
