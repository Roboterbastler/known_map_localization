/*
 * ScoredHypothesis.cpp
 *
 *  Created on: 29.09.2016
 *      Author: jacob
 */

#include <alignment/GpsScoredHypothesis.h>

namespace kml {

GpsScoredHypothesis::GpsScoredHypothesis() :
		gpsSupported(false), supportingHints(0), challengingHints(0) {
}

GpsScoredHypothesis::GpsScoredHypothesis(const Hypothesis &h) :
		Hypothesis(h), gpsSupported(false),
		supportingHints(0), challengingHints(0) {

}

} /* namespace kml */
