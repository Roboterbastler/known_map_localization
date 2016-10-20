/*
 * Hypothesis.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <alignment/Hypothesis.h>

namespace kml {

Hypothesis::Hypothesis() :
		StampedAlignment(), score(0.) {
}

Hypothesis::Hypothesis(const StampedAlignment &a, float score) :
		StampedAlignment(a), score(score) {
}

Hypothesis::~Hypothesis() {

}

} /* namespace kml */
