/*
 * ScoredHypothesis.cpp
 *
 *  Created on: 29.09.2016
 *      Author: jacob
 */

#include <alignment/GpsScoredHypothesis.h>

namespace kml {

GpsScoredHypothesis::GpsScoredHypothesis() : gpsSupported(false) {
}

GpsScoredHypothesis::GpsScoredHypothesis(const Hypothesis &h) : Hypothesis(h), gpsSupported(false) {

}

} /* namespace kml */
