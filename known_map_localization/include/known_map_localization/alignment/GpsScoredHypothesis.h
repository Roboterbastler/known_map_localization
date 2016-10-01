/*
 * ScoredHypothesis.h
 *
 *  Created on: 29.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNMENT_GPSSCOREDHYPOTHESIS_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNMENT_GPSSCOREDHYPOTHESIS_H_

#include <alignment/Hypothesis.h>

namespace kml {

/**
 * Hypothesis used by the GPS filter.
 * Extends the basic hypothesis by a flag telling whether a hypothesis is supported by a GPS hint.
 */
class GpsScoredHypothesis: public Hypothesis {
public:
	GpsScoredHypothesis();

	/**
	 * Copy-constructor from simple hypothesis, setting gpsSupported to false.
	 * @param h The simple hypothesis
	 */
	GpsScoredHypothesis(const Hypothesis &h);

	/// This flag indicates if this hypothesis is supported by at least one GPS constraint
	bool gpsSupported;
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNMENT_GPSSCOREDHYPOTHESIS_H_ */
