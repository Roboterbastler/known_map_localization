/*
 * Filter.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include "Filter.h"

namespace known_map_localization {
namespace filter {

Filter::Filter() : ready(false) {

}

const alignment::Alignment& Filter::getAlignment() const {
	if(!ready) {
		throw AlignmentNotAvailable("Filtered alignment is not yet available");
	}
	return filteredAlignment;
}

} /* namespace filter */
} /* namespace known_map_localization */
