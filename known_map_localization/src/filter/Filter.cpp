/*
 * Filter.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <filter/Filter.h>
#include <Exception.h>

namespace known_map_localization {
namespace filter {

Filter::Filter() : ready(false) {
}

Filter::~Filter() {
}

const alignment::Alignment& Filter::getAlignment() const {
	if(!ready) {
		throw AlignmentNotAvailable("Filtered alignment is not yet available");
	}
	return filteredAlignment;
}

} /* namespace filter */
} /* namespace known_map_localization */
