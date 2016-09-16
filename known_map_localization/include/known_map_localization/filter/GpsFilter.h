/*
 * GpsFilter.h
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_

#include <filter/Filter.h>

namespace known_map_localization {
namespace filter {

/**
 * # GPS Filter
 *
 * ## Parameters
 * - **gps_constraint_radius**: The radius of the constraint given by a GPS fix
 *
 * ## Subscribed topics
 * - __/robot/gps__: The GPS signal
 */
class GpsFilter : public Filter {
public:
	GpsFilter();
	GpsFilter(float constraintRadius, float maxAge, float agingFactor);

	/**
	 * Update the filtered alignment by simply overwriting it with the first new hypothesis.
	 * @param alignment The new alignment
	 */
	void addHypotheses(const alignment::HypothesesVect &hypotheses);

protected:

	/**
	 * Computes a scoring for a given hypothesis.
	 * @param h The hypothesis
	 * @return The score
	 */
	float scoringFunction(const alignment::Hypothesis &h) const;

private:
	/// The score of the filtered alignment
	float filteredAlignmentScore;

	/// The radius of the constraint given by a GPS fix
	float CONSTRAINT_RADIUS;

	float AGING_RATE;

	float CONFIRMATION_FACTOR;
};

} /* namespace filter */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_ */
