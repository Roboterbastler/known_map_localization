/*
 * GpsFilter.cpp
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <known_map_server/KnownMapServer.h>
#include <filter/GpsFilter.h>
#include <base_link/BaseLinkPublisher.h>

#define MAX_HINT_CACHE_SIZE 50

namespace known_map_localization {
namespace filter {

using namespace known_map_server;
using namespace alignment;

GpsFilter::GpsFilter() : filteredAlignmentScore(0) {
	ROS_INFO("    Type: GPS filter");

	ros::NodeHandle nh("~");

	CONSTRAINT_RADIUS = nh.param("gps_constraint_radius", 10.);
	MAX_HINT_AGE = nh.param("max_hint_age", 10.);
	AGING_RATE = nh.param("aging_rate", 1.);
	CONFIRMATION_FACTOR = nh.param("gps_confirmation_factor", 1.05);

	ROS_INFO("    Constraint radius: %.2f", CONSTRAINT_RADIUS);
	ROS_INFO("    Maximum hint age: %.2f", MAX_HINT_AGE);
	ROS_INFO("    Aging rate: %.2f", AGING_RATE);

	gpsSubscriber = nh.subscribe("/robot/gps", 10, &GpsFilter::receiveGpsFix, this);
}

void GpsFilter::addHypotheses(const HypothesesVect &hypotheses) {
	if(ready) {
		// aging factor
		filteredAlignmentScore *= AGING_RATE;
	}

	for(HypothesesVect::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h) {
		float score = scoringFunction(*h);
		if(score > filteredAlignmentScore) {
			filteredAlignment = *h;
			filteredAlignmentScore = score;
			ready = true;
		}
	}
}

float GpsFilter::scoringFunction(const Hypothesis &h) const {
	float score = h.score;

	try {
		tf::Pose copterPose = base_link::BaseLinkPublisher::instance()->getPoseForAlignment(h);
		const tf::Point &copterPosition = copterPose.getOrigin();

		for(std::vector<geometry_msgs::PointStamped>::const_iterator hint = gpsPositionHints.begin(); hint != gpsPositionHints.end(); ++hint) {
			float distance = sqrt(pow(hint->point.x - copterPosition.x(), 2) + pow(hint->point.y - copterPosition.y(), 2));
			float confirmation = distance < CONSTRAINT_RADIUS ? CONFIRMATION_FACTOR : 1.0 / CONFIRMATION_FACTOR;
			float age = (h.stamp - hint->header.stamp).toSec();
			float weight = 1 / MAX_HINT_AGE * age + 1;
			weight = std::min(weight, 1.f);
			weight = std::max(weight, 0.f);
			float confirmedScore = confirmation*score;

			score = weight*confirmedScore + (weight - 1)*score;
		}
	} catch (ScaleNotAvailable &e) {

	} catch(tf::TransformException &e) {

	}

	return score;
}

void GpsFilter::receiveGpsFix(const sensor_msgs::NavSatFix &gpsFix) {
	if(gpsFix.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
		// GPS fix is invalid
		return;
	}

	try {
		geometry_msgs::PointStamped hint = convertGPSPositionToAnchorFrame(gpsFix, *(KnownMapServer::instance()->getAnchor()));
		gpsPositionHints.push_back(hint);
	} catch(DifferentUTMGridZones &e) {
		ROS_WARN("GPS filter: %s.", e.what());
	}

	// drop oldest hints
	while(gpsPositionHints.size() > MAX_HINT_CACHE_SIZE || (!gpsPositionHints.empty() && hintOutdated(gpsPositionHints.front()))) {
		gpsPositionHints.erase(gpsPositionHints.begin());
	}
}

geometry_msgs::PointStamped GpsFilter::convertGPSPositionToAnchorFrame(const sensor_msgs::NavSatFix &gpsFix, const geographic_msgs::GeoPose &anchor) {
	geodesy::UTMPoint utmFix(geodesy::toMsg(gpsFix));
	geodesy::UTMPoint utmAnchor(anchor.position);

	if(!geodesy::sameGridZone(utmFix, utmAnchor)) {
		// for simplicity assume region of operation does not spread over multiple UTM grid zones
		// ignore other GPS fixes
		throw DifferentUTMGridZones("GPS fix lies in a different UTM grid zone than the known map anchor");
	}

	// convert utmFix to anchor frame
	tf::Point anchorFix;
	anchorFix.setX(utmFix.easting - utmAnchor.easting);
	anchorFix.setY(utmFix.northing - utmAnchor.northing);
	anchorFix.setZ(0);

	tf::Quaternion anchorRotation;
	tf::quaternionMsgToTF(anchor.orientation, anchorRotation);
	anchorFix = tf::Transform(anchorRotation.inverse()) * anchorFix;

	geometry_msgs::PointStamped position;
	position.header.frame_id = "/known_map_localization/anchor";
	position.header.stamp = gpsFix.header.stamp;
	tf::pointTFToMsg(anchorFix, position.point);

	return position;
}

bool GpsFilter::hintOutdated(const geometry_msgs::PointStamped &hint) {
	ros::Duration age = ros::Time::now() - hint.header.stamp;
	return age.toSec() > MAX_HINT_AGE;
}

} /* namespace filter */
} /* namespace known_map_localization */
