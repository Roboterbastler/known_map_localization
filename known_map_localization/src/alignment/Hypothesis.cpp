/*
 * Hypothesis.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <alignment/Hypothesis.h>

namespace known_map_localization {
namespace alignment {

Hypothesis::Hypothesis() :
		StampedAlignment(), score(0.) {
}

Hypothesis::Hypothesis(const StampedAlignment &a, float score) :
		StampedAlignment(a), score(score) {
}

} /* namespace alignment */
} /* namespace known_map_localization */
