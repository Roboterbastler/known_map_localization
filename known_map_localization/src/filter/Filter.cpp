/*
 * Filter.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <filter/Filter.h>
#include <filter/PassThroughFilter.h>
#include <filter/GpsFilter.h>
#include <Exception.h>

namespace known_map_localization {
namespace filter {

FilterPtr Filter::_instance;

Filter::Filter() : ready(false) {
	ROS_INFO("Filter initialization...");
}

FilterPtr Filter::instance() {
	if(!_instance) {
		_instance = FilterPtr(new GpsFilter());
	}
	return _instance;
}

Filter::~Filter() {
}

const alignment::Alignment& Filter::getAlignment() const {
	if(!ready) {
		throw AlignmentNotAvailable("Filtered alignment is not yet available");
	}
	return filteredAlignment;
}

bool Filter::isAvailable() const {
	return ready;
}

} /* namespace filter */
} /* namespace known_map_localization */
