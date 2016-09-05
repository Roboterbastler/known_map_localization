/*
 * SlamScaleManager.cpp
 *
 *  Created on: 27.08.2016
 *      Author: jacob
 */

#include <ros/ros.h>
#include <geodesy/wgs84.h>

#include <SlamScaleManager.h>
#include <known_map_server/KnownMapServer.h>
#include <Exception.h>

namespace known_map_localization {

namespace slam_scale_manager {

SlamScaleManagerPtr SlamScaleManager::_instance;

SlamScaleManager::SlamScaleManager() :
		mode(determineMode()), isValid(false), scale(1.0) {
	ros::NodeHandle nh("~");

	switch (mode) {
	case PARAMETER:
		ROS_INFO("SLAM scale manager mode: PARAMETER");
		if (nh.getParam("slam_map_scale", scale)) {
			ROS_INFO("Setting scale to %f", scale);
			isValid = true;
		} else {
			ROS_FATAL("No SLAM scale parameter found.");
			ros::shutdown();
			return;
		}
		break;
	case ALIGNMENT:
		ROS_INFO("SLAM scale manager mode: ALIGNMENT");
		break;
	case GPS:
		ROS_INFO("SLAM scale manager mode: GPS");
		gpsFixSubscriber = nh.subscribe("/fix", 100,
				&SlamScaleManager::receiveGpsFix, this);
		break;
	default:
		ROS_FATAL("Illegal or not specified SLAM scale manager mode.");
		ros::shutdown();
		return;
	}
}

SlamScaleManagerPtr SlamScaleManager::instance() {
	if(!_instance) {
		_instance = SlamScaleManagerPtr(new SlamScaleManager());
	}
	return _instance;
}

SlamScaleMode SlamScaleManager::determineMode() const {
	ros::NodeHandle nh("~");
	std::string invalid = "not specified";
	std::string mode = nh.param("slam_scale_mode", invalid);
	std::transform(mode.begin(), mode.end(),mode.begin(), ::toupper);

	if(mode == "PARAMETER") {
		return PARAMETER;
	}
	if(mode =="ALIGNMENT") {
		return ALIGNMENT;
	}
	if(mode == "GPS") {
		return GPS;
	}
	return INVALID;
}

float SlamScaleManager::getSlamScale() const {
	if (isValid) {
		return scale;
	} else {
		throw ScaleNotAvailable("Scale estimate not available");
	}
}

void SlamScaleManager::updateSlamScale(float scale) {
	if (mode == ALIGNMENT) {
		this->scale = scale;
		isValid = true;
	}
}

tf::Transform SlamScaleManager::convertTransform(
		tf::Transform transform) const {
	transform.getOrigin() *= getSlamScale();
	return transform;
}

tf::StampedTransform SlamScaleManager::convertTransform(
		tf::StampedTransform transform) const {
	transform.setData(convertTransform((tf::Transform) transform));
	return transform;
}

geometry_msgs::Pose SlamScaleManager::convertPoseMsg(
		geometry_msgs::Pose pose) const {
	float scale = getSlamScale();
	pose.position.x *= scale;
	pose.position.y *= scale;
	pose.position.z *= scale;
	return pose;
}

void SlamScaleManager::receiveGpsFix(const sensor_msgs::NavSatFix &fix) {
	if (fix.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
		// valid fix
		// get SLAM position in anchor frame
		tf::StampedTransform slamTransformation;
		try {
			listener.lookupTransform("/known_map_localization/anchor",
					"ORB_base_link", fix.header.stamp, slamTransformation);
		} catch (tf::TransformException &e) {
			ROS_DEBUG_THROTTLE(1,
					"SLAM scale manager: SLAM position not available.");
			return;
		}
		geometry_msgs::Pose slamPose;
		tf::poseTFToMsg(slamTransformation, slamPose);
		geometry_msgs::Point slamPosition = slamPose.position;

		geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(fix);
		geodesy::UTMPoint utmPoint(geoPoint);
		geodesy::UTMPoint utmAnchor(known_map_server::KnownMapServer::instance()->getAnchor()->position);

		if (!geodesy::sameGridZone(utmAnchor, utmPoint)) {
			ROS_ERROR_ONCE(
					"UTM zone of GPS fix does not match zone of known map anchor.");
			return;
		}
		// GPS position in anchor frame
		geodesy::UTMPoint gpsPosition(utmPoint.easting - utmAnchor.easting,
				utmPoint.northing - utmAnchor.northing, utmPoint.zone,
				utmPoint.band);

		// remove all positions where the UTM zone does not match the new GPS fix's zone
		std::remove_if(pointData.begin(), pointData.end(), PositionUTMZoneFilter(gpsPosition.zone, gpsPosition.band));
		pointData.push_back(PositionPair(gpsPosition, slamPosition));

		leastSquares(pointData);
	}
}

void SlamScaleManager::leastSquares(std::vector<PositionPair> pointData) {

}

}

} /* namespace known_map_localization */
