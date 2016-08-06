/*
 * PassThroughFilter.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <filter/PassThroughFilter.h>

namespace known_map_localization {
namespace filter {

void PassThroughFilter::addAlignment(alignment::StampedAlignment alignment) {
	filteredAlignment = alignment;
	ready = true;
}

} /* namespace filter */
} /* namespace known_map_localization */
