/*
 * PassThroughFilter.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <filter/PassThroughFilter.h>
#include <SlamScaleManager.h>
#include <logging/DataLogger.h>

namespace known_map_localization {
namespace filter {

PassThroughFilter::PassThroughFilter() {
	ROS_INFO("    Type: Pass through filter");
}

void PassThroughFilter::addHypotheses(const alignment::HypothesesVect &hypotheses) {
	if(hypotheses.empty()) {
		return;
	}

	filteredAlignment = hypotheses.front();
	SlamScaleManager::instance()->updateSlamScale(filteredAlignment.scale);
	logging::DataLogger::instance()->logFilter(filteredAlignment);
	ready = true;
}

} /* namespace filter */
} /* namespace known_map_localization */
