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

namespace known_map_localization {
namespace filter {

using namespace known_map_server;

GpsFilter::GpsFilter() {
	ros::NodeHandle nh("~");

	gpsSubscriber = nh.subscribe("/robot/gps", 10, &GpsFilter::receiveGpsFix, this);
}

void GpsFilter::addHypotheses(const alignment::HypothesesVect &hypotheses) {
	// TODO
}

void GpsFilter::receiveGpsFix(const sensor_msgs::NavSatFix &gpsFix) {
	if(gpsFix.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
		// GPS fix is invalid
		return;
	}

	// TODO
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

} /* namespace filter */
} /* namespace known_map_localization */
