/*
 * PassThroughFilter.h
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_PASSTHROUGHFILTER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_PASSTHROUGHFILTER_H_

#include <filter/Filter.h>

namespace known_map_localization {
namespace filter {

/**
 * # PassThroughFilter
 *
 * Default filter that just passes through the last added alignment.
 */
class PassThroughFilter: public Filter {
public:
	/**
	 * Update the filtered alignment by simply overwriting it with the first new hypothesis.
	 * @param alignment The new alignment
	 */
	void addHypotheses(const alignment::HypothesesVect &hypotheses);
};

} /* namespace filter */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_PASSTHROUGHFILTER_H_ */
