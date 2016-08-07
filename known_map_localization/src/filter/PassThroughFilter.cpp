/*
 * PassThroughFilter.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <filter/PassThroughFilter.h>

namespace known_map_localization {
namespace filter {

void PassThroughFilter::addHypotheses(const alignment::HypothesesVect &hypotheses) {
	if(hypotheses.empty()) {
		return;
	}

	filteredAlignment = *(hypotheses.begin());
	ready = true;
}

} /* namespace filter */
} /* namespace known_map_localization */
