/*
 * Hypothesis.h
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_HYPOTHESIS_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_HYPOTHESIS_H_

#include <alignment/StampedAlignment.h>

namespace kml {

/**
 * # Hypothesis
 * This class encapsulates a hypothesis, that can be returned by an aligning algorithm.
 * In addition to an alignment it contains a score too and provides comparison operators
 * based on that score.
 */
class Hypothesis: public StampedAlignment {
public:
	Hypothesis();
	Hypothesis(const StampedAlignment &a, float score = 0.);
	virtual ~Hypothesis();

	/**
	 * Greater-then check based on the scores.
	 * @param other The other hypothesis
	 * @return True if the other score is greater
	 */
	bool operator<(const Hypothesis &other) const;

	/**
	 * Equality check based on the scores.
	 * @param other The other hypothesis
	 * @return True if scores are equal
	 */
	bool operator==(const Hypothesis &other) const;

	/// Score as an estimate of certainty or quality of this hypothesis. Greater means better.
	float score;
};

typedef std::vector<Hypothesis> HypothesesVect;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_HYPOTHESIS_H_ */
