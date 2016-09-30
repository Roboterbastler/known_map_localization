/*
 * DecayFilter.cpp
 *
 *  Created on: 30.09.2016
 *      Author: jacob
 */

#include <filter/DecayFilter.h>

#include <ros/ros.h>

#include <logging/DataLogger.h>

namespace known_map_localization {
namespace filter {

using namespace alignment;

DecayFilter::DecayFilter() : mScore_(0) {
	ros::NodeHandle nh("~");

	// default: decay disabled
	mDecayFactor_ = nh.param("decay_factor", 1.0);

	ROS_INFO("    Decay factor: %.2f", mDecayFactor_);
}

void DecayFilter::scoreDecay() {
	mScore_ *= mDecayFactor_;
}

void DecayFilter::addHypotheses(const HypothesesVect &hypotheses) {
	// degrade score by small factor
	scoreDecay();

	ROS_DEBUG("Checking %ld new hypotheses. Current filtered alignment score: %f", hypotheses.size(), mScore_);

	Hypothesis betterHypothesis;
	if(checkForBetterHypothesis(hypotheses, betterHypothesis)) {
		filteredAlignment = betterHypothesis;
		mScore_ = betterHypothesis.score;
		ready = true;

		logging::DataLogger::instance()->logFilter(filteredAlignment);
	}
}

bool DecayFilter::checkForBetterHypothesis(const HypothesesVect &hypotheses, Hypothesis &betterHypothesis) const {
	bool foundBetterHypothesis = false;

	for(HypothesesVect::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h) {
		ROS_DEBUG("  - Hypothesis: x = %.2f, y = %.2f theta = %.2f score = %.4f", h->x, h->y, h->theta, h->score);

		if(isBetter(*h)) {
			betterHypothesis = *h;
			foundBetterHypothesis = true;

			ROS_DEBUG("    -> Found better hypothesis. New filtered alignment.");
		}
	}
	return foundBetterHypothesis;
}

bool DecayFilter::isBetter(const Hypothesis &hypothesis) const {
	return hypothesis.score >= mScore_;
}

} /* namespace filter */
} /* namespace known_map_localization */
