/*
 * GpsManager.cpp
 *
 *  Created on: 16.09.2016
 *      Author: jacob
 */

#include <GpsManager.h>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <visualization_msgs/Marker.h>

#include <Exception.h>
#include <known_map_server/KnownMapServer.h>
#include <SlamScaleManager.h>

#define MAX_KEYPOINT_SIZE 10
#define MAX_QUEUE_SIZE 10

namespace known_map_localization {

using namespace known_map_server;

GpsManagerPtr GpsManager::_instance;

GpsManager::GpsManager() {
	ROS_INFO("GPS manager initialization...");

	ros::WallDuration interval(1.0);
	ROS_INFO("    Interval: %.2f", interval.toSec());

	ros::NodeHandle nh("~");

	gpsFixSubscriber = nh.subscribe("/gps_fix", 1000, &GpsManager::receiveGpsFix, this);
	timer = nh.createWallTimer(interval, &GpsManager::updateKeyPoints, this);
	gpsPositionMarkerPublisher = nh.advertise<visualization_msgs::Marker>("gps_position_marker", 1);
	gpsHintsUpdatedPublisher = nh.advertise<std_msgs::Empty>("gps_hints_updated", 1);
}

void GpsManager::receiveGpsFix(const sensor_msgs::NavSatFix &fix) {
	if(fix.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
		// GPS fix is invalid
		return;
	}

	try {
		GpsHint hint;
		hint.gpsPosition = convertGPSPositionToAnchorFrame(fix, *(KnownMapServer::instance()->getAnchor()));
		hint.stamp = fix.header.stamp;
		hint.gpsFix = fix;

		hintQueue.push_back(hint);

		// remove front if queue too full
		if(hintQueue.size() > MAX_QUEUE_SIZE) {
			hintQueue.erase(hintQueue.begin());
		}

		publishGpsFixMarker();

		ROS_DEBUG("Received new GPS fix. Queue size: %ld", hintQueue.size());
	} catch(DifferentUTMGridZones &e) {
		ROS_WARN("GPS Manager unsupported fix: %s", e.what());
	}
}

void GpsManager::updateKeyPoints(const ros::WallTimerEvent& event) {
	bool modified = false;

	for(GpsHintVect::iterator hint = hintQueue.begin(); hint != hintQueue.end();) {
		try {
			GpsKeyPoint kp(*hint);
			kp.baseLink = getSlamBaseLink(kp.stamp);

			// add new key point
			keypoints.push_back(kp);
			modified = true;

			// remove GPS fix from queue
			hint = hintQueue.erase(hint);
		} catch (tf::TransformException &e) {
			ROS_DEBUG("SLAM base link not found for GPS hint: %s", e.what());
			++hint;
		}
	}

	// remove front if queue too full
	while(keypoints.size() > MAX_KEYPOINT_SIZE) {
		keypoints.erase(keypoints.begin());
		modified = true;
	}

	publishKeyPointMarker();

	if (modified) {
		gpsHintsUpdatedPublisher.publish(std_msgs::Empty());
	}

	ROS_DEBUG("Key points: %ld", keypoints.size());
}

tf::StampedTransform GpsManager::getSlamBaseLink(ros::Time t) {
	tf::StampedTransform slamBaseLink;
	listener.lookupTransform("orb_slam/map", "ORB_base_link", t, slamBaseLink);
	return slamBaseLink;
}

geometry_msgs::Point GpsManager::convertGPSPositionToAnchorFrame(const sensor_msgs::NavSatFix &gpsFix, const geographic_msgs::GeoPose &anchor) {
	geodesy::UTMPoint utmFix(geodesy::toMsg(gpsFix));
	geodesy::UTMPoint utmAnchor(anchor.position);

	if(!geodesy::sameGridZone(utmFix, utmAnchor)) {
		// for simplicity assume region of operation does not spread over multiple UTM grid zones
		// ignore other GPS fixes
		std::stringstream errorMessage;
		errorMessage << "GPS fix lies in a different UTM grid zone (" << int(utmFix.zone) << utmFix.band
				<< ") than the known map anchor (" << int(utmAnchor.zone) << utmAnchor.band << ")";
		throw DifferentUTMGridZones(errorMessage.str());
	}

	// convert utmFix to anchor frame
	tf::Point anchorFix;
	anchorFix.setX(utmFix.easting - utmAnchor.easting);
	anchorFix.setY(utmFix.northing - utmAnchor.northing);
	anchorFix.setZ(0);

	tf::Quaternion anchorRotation;
	tf::quaternionMsgToTF(anchor.orientation, anchorRotation);
	anchorFix = tf::Transform(anchorRotation.inverse()) * anchorFix;

	geometry_msgs::Point position;
	tf::pointTFToMsg(anchorFix, position);

	return position;
}

const GpsKeyPointVect& GpsManager::getKeyPoints() const {
	return keypoints;
}

GpsManagerPtr GpsManager::instance() {
	if(!_instance) {
		_instance = GpsManagerPtr(new GpsManager());
	}
	return _instance;
}

geometry_msgs::Pose GpsKeyPoint::getRobotPose(const alignment::Alignment &alignment) const {
	tf::Transform slamMapFrame_to_knownMapFrame = alignment.toTfTransform();

	// convert pose to real world scale
	tf::Transform slamMapFrame_to_slamBaseLink = SlamScaleManager::instance()->convertTransform(baseLink);

	tf::Pose pose;
	pose.setIdentity();
	pose *= slamMapFrame_to_knownMapFrame.inverse();
	pose *= slamMapFrame_to_slamBaseLink;

	geometry_msgs::Pose poseMsg;
	tf::poseTFToMsg(pose, poseMsg);
	return poseMsg;
}

void GpsManager::publishKeyPointMarker() {
	visualization_msgs::Marker keyPointMarker;
	keyPointMarker.header.frame_id = KnownMapServer::instance()->getKnownMap()->header.frame_id;
	keyPointMarker.header.stamp = ros::Time::now();
	keyPointMarker.ns = "GPS-Positions";
	keyPointMarker.id = 1;
	keyPointMarker.frame_locked = true;
	keyPointMarker.type = visualization_msgs::Marker::POINTS;
	keyPointMarker.action = visualization_msgs::Marker::ADD;
	keyPointMarker.color.g = 1.0;
	keyPointMarker.color.a = 1.0;
	float pointSize = 0.2;
	keyPointMarker.scale.x = pointSize;
	keyPointMarker.scale.y = pointSize;
	keyPointMarker.pose.orientation.w = 1.0;

	for(GpsKeyPointVect::const_iterator keyPoint = keypoints.begin(); keyPoint != keypoints.end(); ++keyPoint) {
		keyPointMarker.points.push_back(keyPoint->gpsPosition);
	}

	gpsPositionMarkerPublisher.publish(keyPointMarker);
}

void GpsManager::publishGpsFixMarker() {
	if(gpsPositionMarkerPublisher.getNumSubscribers() > 0) {
		visualization_msgs::Marker gpsFixMarker;
		gpsFixMarker.header.frame_id = KnownMapServer::instance()->getKnownMap()->header.frame_id;
		gpsFixMarker.header.stamp = ros::Time::now();
		gpsFixMarker.ns = "GPS-Fixes";
		gpsFixMarker.id = 0;
		gpsFixMarker.frame_locked = true;
		gpsFixMarker.type = visualization_msgs::Marker::POINTS;
		gpsFixMarker.action = visualization_msgs::Marker::ADD;
		gpsFixMarker.color.b = 1.0;
		gpsFixMarker.color.a = 1.0;
		float pointSize = 0.2;
		gpsFixMarker.scale.x = pointSize;
		gpsFixMarker.scale.y = pointSize;
		gpsFixMarker.pose.orientation.w = 1.0;
		for(GpsHintVect::const_iterator hint = hintQueue.begin(); hint != hintQueue.end(); ++hint) {
			gpsFixMarker.points.push_back(hint->gpsPosition);
		}
		gpsPositionMarkerPublisher.publish(gpsFixMarker);
	}
}

GpsKeyPoint::GpsKeyPoint(const GpsHint &hint) :
		GpsHint(hint) {
}

double GpsHint::distanceTo(const geometry_msgs::Point &p) const {
	return sqrt(pow(gpsPosition.x - p.x, 2) + pow(gpsPosition.y - p.y, 2));
}

} /* namespace known_map_localization */
