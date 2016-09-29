/*
 * ScoredHypothesis.cpp
 *
 *  Created on: 29.09.2016
 *      Author: jacob
 */

#include <alignment/GpsScoredHypothesis.h>

namespace known_map_localization {
namespace alignment {

GpsScoredHypothesis::GpsScoredHypothesis() : gpsSupported(false) {
}

GpsScoredHypothesis::GpsScoredHypothesis(const Hypothesis &h) : Hypothesis(h), gpsSupported(false) {

}

} /* namespace alignment */
} /* namespace known_map_localization */
