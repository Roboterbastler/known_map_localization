/*
 * GpsFilter.cpp
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#include <filter/GpsFilter.h>

#include <Exception.h>
#include <GpsManager.h>

#define MAX_HINT_CACHE_SIZE 10

namespace known_map_localization {
namespace filter {

using namespace alignment;

GpsFilter::GpsFilter() : filteredAlignmentScore(0) {
	ROS_INFO("    Type: GPS filter");

	ros::NodeHandle nh("~");

	CONSTRAINT_RADIUS = nh.param("gps_constraint_radius", 10.);
	AGING_RATE = nh.param("aging_rate", 1.);
	CONFIRMATION_FACTOR = nh.param("gps_confirmation_factor", 1.05);

	ROS_INFO("    Constraint radius: %.2f", CONSTRAINT_RADIUS);
	ROS_INFO("    Aging rate: %.2f", AGING_RATE);
}

void GpsFilter::addHypotheses(const HypothesesVect &hypotheses) {
	if(ready) {
		// aging factor
		filteredAlignmentScore *= AGING_RATE;
	}

	ROS_DEBUG("Adding %ld new hypotheses...", hypotheses.size());
	ROS_DEBUG("Current filtered alignment score: %f", filteredAlignmentScore);

	for(HypothesesVect::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h) {
		float score = scoringFunction(*h);

		ROS_DEBUG("      -> Hypothesis has score: %f", score);

		if(score > filteredAlignmentScore) {
			filteredAlignment = *h;
			filteredAlignmentScore = score;
			ready = true;
			ROS_DEBUG("        -> New best alignment.");
		}
	}
}

float GpsFilter::scoringFunction(const Hypothesis &h) const {
	float score = h.score;

	try {
		const GpsKeyPointVect &gpsHints = GpsManager::instance()->getKeyPoints();
		for(GpsKeyPointVect::const_iterator hint = gpsHints.begin(); hint != gpsHints.end(); ++hint) {
			geometry_msgs::Pose robotPose = hint->getRobotPose(h);
			float distance = sqrt(pow(hint->gpsPosition.x - robotPose.position.x, 2) + pow(hint->gpsPosition.y - robotPose.position.y, 2));
			float confirmation = distance < CONSTRAINT_RADIUS ? CONFIRMATION_FACTOR : 1.0 / CONFIRMATION_FACTOR;
			score = confirmation * score;

			ROS_DEBUG("    - Distance: %f  Confirmation: %f  Confirmed Score: %f  Alignment Score: %f", distance, confirmation, score, h.score);
		}
	} catch (ScaleNotAvailable &e) {
		ROS_DEBUG("Scoring not possible: %s", e.what());
	} catch(tf::TransformException &e) {
		ROS_DEBUG("Scoring not possible: tf lookup failed (%s)", e.what());
	}

	return score;
}

} /* namespace filter */
} /* namespace known_map_localization */
