/*
 * GpsFilter.cpp
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#include <filter/GpsFilter.h>

#include <known_map_server/KnownMapServer.h>
#include <Exception.h>
#include <GpsManager.h>

#define MAX_HINT_CACHE_SIZE 10

namespace known_map_localization {
namespace filter {

using namespace known_map_server;
using namespace alignment;

GpsFilter::GpsFilter() : filteredAlignmentScore(0) {
	ROS_INFO("    Type: GPS filter");

	ros::NodeHandle nh("~");

	gpsConstraintsMarkerPublisher = nh.advertise<visualization_msgs::Marker>("gps_position_marker", 1);

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

	visualization_msgs::Marker constraintsMarker;
	constraintsMarker.header.frame_id = KnownMapServer::instance()->getKnownMap()->header.frame_id;
	constraintsMarker.header.stamp = ros::Time::now();
	constraintsMarker.ns = "GPS-Constraints";
	constraintsMarker.id = 2;
	constraintsMarker.frame_locked = true;
	constraintsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
	constraintsMarker.action = visualization_msgs::Marker::ADD;
	constraintsMarker.scale.x = CONSTRAINT_RADIUS;
	constraintsMarker.scale.y = CONSTRAINT_RADIUS;
	constraintsMarker.scale.z = CONSTRAINT_RADIUS;
	constraintsMarker.pose.orientation.w = 1.0;
	constraintsMarker.color.a = 0.5;

	for(HypothesesVect::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h) {
		visualization_msgs::Marker hypothesisConstraints;
		float score = scoringFunction(*h, hypothesisConstraints);

		ROS_DEBUG("      -> Hypothesis has score: %f", score);

		if(score > filteredAlignmentScore) {
			filteredAlignment = *h;
			filteredAlignmentScore = score;
			ready = true;

			constraintsMarker.points = hypothesisConstraints.points;
			constraintsMarker.colors = hypothesisConstraints.colors;
			ROS_DEBUG("        -> New best alignment.");
		}
	}

	gpsConstraintsMarkerPublisher.publish(constraintsMarker);
}

float GpsFilter::scoringFunction(const Hypothesis &h, visualization_msgs::Marker &constraints) const {
	float score = h.score;

	try {
		const GpsKeyPointVect &gpsHints = GpsManager::instance()->getKeyPoints();
		for(GpsKeyPointVect::const_iterator hint = gpsHints.begin(); hint != gpsHints.end(); ++hint) {
			geometry_msgs::Pose robotPose = hint->getRobotPose(h);
			float distance = sqrt(pow(hint->gpsPosition.x - robotPose.position.x, 2) + pow(hint->gpsPosition.y - robotPose.position.y, 2));
			float confirmation = distance < CONSTRAINT_RADIUS ? CONFIRMATION_FACTOR : 1.0 / CONFIRMATION_FACTOR;
			score = confirmation * score;

			constraints.points.push_back(hint->gpsPosition);
			std_msgs::ColorRGBA color;
			color.a = 0.5;
			if(distance < CONSTRAINT_RADIUS) {
				color.g = 1.;
			} else {
				color.r = 1.;
			}
			constraints.colors.push_back(color);
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
