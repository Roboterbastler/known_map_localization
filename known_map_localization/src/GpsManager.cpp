/*
 * GpsManager.cpp
 *
 *  Created on: 16.09.2016
 *      Author: jacob
 */

#include <GpsManager.h>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <Exception.h>
#include <known_map_server/KnownMapServer.h>
#include <SlamScaleManager.h>

#define MAX_KEYPOINT_SIZE 10

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
}

void GpsManager::receiveGpsFix(const sensor_msgs::NavSatFix &fix) {
	if(fix.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
		// GPS fix is invalid
		return;
	}

	// TODO: check minimal time difference or spatial distance

	fixQueue.push_back(fix);
	ROS_DEBUG("Received new GPS fix. Queue size: %ld", fixQueue.size());
}

void GpsManager::updateKeyPoints(const ros::WallTimerEvent& event) {
	for(std::vector<sensor_msgs::NavSatFix>::iterator fix = fixQueue.begin(); fix != fixQueue.end();) {
		try {
			GpsKeyPoint kp;
			kp.baseLink = getSlamBaseLink(fix->header.stamp);

			kp.stamp = fix->header.stamp;

			kp.gpsFix = *fix;
			kp.gpsPosition = convertGPSPositionToAnchorFrame(*fix, *(KnownMapServer::instance()->getAnchor()));

			// add new key point
			keypoints.push_back(kp);

			// remove GPS fix from queue
			fix = fixQueue.erase(fix);
		} catch (tf::TransformException &e) {
			++fix;
		} catch (DifferentUTMGridZones &e) {
			ROS_WARN("GPS Manager unsupported fix: %s", e.what());
			// remove unsupported GPS fix from queue
			fix = fixQueue.erase(fix);
		}
	}

	// remove front if queue too full
	while(keypoints.size() > MAX_KEYPOINT_SIZE) {
		keypoints.erase(keypoints.begin());
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

} /* namespace known_map_localization */
