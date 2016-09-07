/*
 * SlamScaleManager.cpp
 *
 *  Created on: 27.08.2016
 *      Author: jacob
 */

#include <ros/ros.h>
#include <geodesy/wgs84.h>

#include <SlamScaleManager.h>
#include <Exception.h>

namespace known_map_localization {

typedef std::vector<PositionPair>::iterator PositionPairIter;
typedef std::vector<PositionPair>::const_iterator PositionPairConstIter;

SlamScaleManagerPtr SlamScaleManager::_instance;

SlamScaleManager::SlamScaleManager() :
		mode(determineMode()), isValid(false), scale(1.0) {
	ros::NodeHandle nh("~");

	ROS_INFO("SLAM scale manager initialization...");

	switch (mode) {
	case PARAMETER:
		ROS_INFO("    Mode: PARAMETER");
		if (nh.getParam("slam_map_scale", scale)) {
			ROS_INFO("    Scale: %f", scale);
			isValid = true;
		} else {
			ROS_FATAL("No SLAM scale parameter found.");
			ros::shutdown();
			return;
		}
		break;
	case ALIGNMENT:
		ROS_INFO("    Mode: ALIGNMENT");
		break;
	case GPS:
		ROS_INFO("    Mode: GPS");
		gpsFixSubscriber = nh.subscribe("/fix", 100,
				&SlamScaleManager::receiveGpsFix, this);
		break;
	default:
		ROS_FATAL("    Illegal or not specified SLAM scale manager mode.");
		ros::shutdown();
		return;
	}
}

SlamScaleMode SlamScaleManager::getMode() const {
	return mode;
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
		geodesy::UTMPoint utmGpsPoint(geodesy::toMsg(fix));

		// remove all positions where the UTM zone does not match the new GPS fix's zone
		std::remove_if(pointData.begin(), pointData.end(), PositionUTMZoneFilter(utmGpsPoint.zone, utmGpsPoint.band));

		pointData.push_back(PositionPair(utmGpsPoint, slamPose.position));

		estimateScale(pointData);
	}
}

void SlamScaleManager::estimateScale(const std::vector<PositionPair> &pointData) {
	if(pointData.size() >= 2) {
		float realWorldDistance = 0;
		float slamMapDistance;

		for(PositionPairConstIter first = pointData.begin(); first != pointData.end(); ++first) {
			for(PositionPairConstIter second = pointData.begin(); second != pointData.end(); ++second) {
				if(distance(first->first, second->first) > realWorldDistance) {
					realWorldDistance = distance(first->first, second->first);
					slamMapDistance = distance(first->second, second->second);
				}
			}
		};

		scale = realWorldDistance / slamMapDistance;
	}
}

float SlamScaleManager::distance(const geodesy::UTMPoint &p1, const geodesy::UTMPoint &p2) {
	return sqrt(pow(p1.easting - p2.easting, 2.) + pow(p1.northing - p2.northing, 2.));
}

float SlamScaleManager::distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
	return sqrt(pow(p1.x - p2.x, 2.) + pow(p1.y - p2.y, 2.));
}

} /* namespace known_map_localization */
