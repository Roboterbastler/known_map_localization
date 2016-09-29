/*
 * GpsFilter.cpp
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#include <filter/GpsFilter.h>

#include <known_map_server/KnownMapServer.h>
#include <Exception.h>
#include <logging/DataLogger.h>

#define MAX_HINT_CACHE_SIZE 10

namespace known_map_localization {
namespace filter {

using namespace known_map_server;
using namespace alignment;

GpsFilter::GpsFilter() : constraintsMarker(setUpContraintMarker()) {
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
	bool filteredHypothesisModified = false;

	if(ready) {
		// small degradation factor
		filteredHypothesis.score *= AGING_RATE;
	}

	ROS_DEBUG("Checking %ld new hypotheses. Current filtered alignment score: %f", hypotheses.size(), filteredHypothesis.score);

	// score all new hypotheses
	for(HypothesesVect::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h) {
		visualization_msgs::Marker hypothesisConstraints = setUpContraintMarker();

		GpsScoredHypothesis scoredHypothesis(*h);

		// compute a score
		scoringFunction(scoredHypothesis, hypothesisConstraints);

		ROS_DEBUG("      -> Hypothesis has score: %f", scoredHypothesis.score);

		if(scoredHypothesis.score > filteredHypothesis.score) {
			// always prefer GPS-supported hypotheses
			if(!filteredHypothesis.gpsSupported || scoredHypothesis.gpsSupported) {
				filteredHypothesis = scoredHypothesis;
				filteredHypothesisModified = true;
				ready = true;

				constraintsMarker = hypothesisConstraints;
				ROS_DEBUG("        -> New best alignment.");
			} else {
				ROS_DEBUG("        -> Rejected, because current filtered hypothesis is supported by GPS hints.");
			}
		}
	}

	if(filteredHypothesisModified) {
		logging::DataLogger::instance()->logFilter(filteredHypothesis);
	}

	gpsConstraintsMarkerPublisher.publish(constraintsMarker);
}

void GpsFilter::scoringFunction(GpsScoredHypothesis &h, visualization_msgs::Marker &constraints) const {
	try {
		const GpsKeyPointVect &gpsHints = GpsManager::instance()->getKeyPoints();
		for(GpsKeyPointVect::const_iterator hint = gpsHints.begin(); hint != gpsHints.end(); ++hint) {
			geometry_msgs::Pose robotPose = hint->getRobotPose(h);
			float distance = sqrt(pow(hint->gpsPosition.x - robotPose.position.x, 2) + pow(hint->gpsPosition.y - robotPose.position.y, 2));
			bool supportingConstraint = distance < CONSTRAINT_RADIUS;
			float confirmation = supportingConstraint ? CONFIRMATION_FACTOR : 1.0 / CONFIRMATION_FACTOR;

			h.score = confirmation * h.score;
			h.gpsSupported |= supportingConstraint;

			addConstraintMarker(*hint, constraints, supportingConstraint);

			ROS_DEBUG("    - Distance: %f  Confirmation: %f  Confirmed Score: %f", distance, confirmation, h.score);
		}
	} catch (ScaleNotAvailable &e) {
		ROS_DEBUG("Scoring not possible: %s", e.what());
	} catch(tf::TransformException &e) {
		ROS_DEBUG("Scoring not possible: tf lookup failed (%s)", e.what());
	}
}

visualization_msgs::Marker GpsFilter::setUpContraintMarker() const {
	visualization_msgs::Marker marker;
	marker.header.frame_id = KnownMapServer::instance()->getKnownMap()->header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "GPS-Constraints";
	marker.id = 2;
	marker.frame_locked = true;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = CONSTRAINT_RADIUS;
	marker.scale.y = CONSTRAINT_RADIUS;
	marker.scale.z = CONSTRAINT_RADIUS;
	marker.pose.orientation.w = 1.0;
	marker.color.a = 0.5;
	return marker;
}

void GpsFilter::addConstraintMarker(const GpsKeyPoint &hint, visualization_msgs::Marker &marker, bool supporting) const {
	std_msgs::ColorRGBA color;
	color.a = 0.5;
	if(supporting) {
		color.g = 1.;
	} else {
		color.r = 1.;
	}
	marker.colors.push_back(color);
	marker.points.push_back(hint.gpsPosition);
}

const alignment::Alignment& GpsFilter::getAlignment() const {
	return filteredHypothesis;
}

} /* namespace filter */
} /* namespace known_map_localization */
