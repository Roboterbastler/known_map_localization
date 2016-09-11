/*
 * SlamScaleManager.cpp
 *
 *  Created on: 27.08.2016
 *      Author: jacob
 */

#include <numeric>

#include <ros/ros.h>
#include <geodesy/wgs84.h>

#include <SlamScaleManager.h>
#include <Exception.h>

#define MAX_POSITION_CACHE_SIZE 100
#define MIN_GPS_DISTANCE_M 0.5

namespace known_map_localization {

typedef std::vector<PositionElement>::const_iterator PositionElementConstIter;

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
		gpsFixSubscriber = nh.subscribe("/robot/gps", 10,
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
	if (!_instance) {
		_instance = SlamScaleManagerPtr(new SlamScaleManager());
	}
	return _instance;
}

SlamScaleMode SlamScaleManager::determineMode() const {
	ros::NodeHandle nh("~");
	std::string invalid = "not specified";
	std::string mode = nh.param("slam_scale_mode", invalid);
	std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);

	if (mode == "PARAMETER") {
		return PARAMETER;
	}
	if (mode == "ALIGNMENT") {
		return ALIGNMENT;
	}
	if (mode == "GPS") {
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
	if (fix.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
		// fix is not valid
		return;
	}

	geodesy::UTMPoint utmGpsPoint(geodesy::toMsg(fix));

	if (!positionCache.empty() && geodesy::sameGridZone(utmGpsPoint, positionCache.back().gps)) {
		if (distance(utmGpsPoint, positionCache.back().gps) < MIN_GPS_DISTANCE_M) {
			// too small distance, skip GPS fix
			return;
		}
	}

	tf::StampedTransform slamTransformation;
	try {
		if (!listener.waitForTransform("/orb_slam/map", "ORB_base_link",
				fix.header.stamp, ros::Duration(1.0))) {
			ROS_DEBUG("Waited for transform, but it didn't become available.");
			return;
		}
		listener.lookupTransform("/orb_slam/map", "ORB_base_link",
				fix.header.stamp, slamTransformation);
	} catch (tf::TransformException &e) {
		ROS_WARN("SLAM scale manager: SLAM position not available (%s).",
				e.what());
		return;
	}

	geometry_msgs::Pose slamPose;
	tf::poseTFToMsg(slamTransformation, slamPose);

	if (!positionCache.empty()) {
		const geodesy::UTMPoint &first = positionCache.front().gps;
		if (!geodesy::sameGridZone(first, utmGpsPoint)) {
			// copter crossed border between two different UTM zones
			// reset position cache
			positionCache.clear();
			ROS_INFO("SLAM scale manager: UTM zone border crossed. Resetting GPS position cache.");
		}
	}

	// add new position cache element
	positionCache.push_back(PositionElement(utmGpsPoint, slamPose.position));
	ROS_DEBUG("Position cache size: %ld", positionCache.size());

	// remove oldest element if too big
	if (positionCache.size() > MAX_POSITION_CACHE_SIZE) {
		positionCache.erase(positionCache.begin());
		ROS_DEBUG_ONCE(
				"SLAM scale manager: Reached maximum GPS position cache size. Starting to remove oldest positions.");
	}

	estimateScale(positionCache);
}

void SlamScaleManager::estimateScale(
		const std::vector<PositionElement> &pointData) {
	if (pointData.size() >= 2) {
		std::vector<double> scales;

		for (PositionElementConstIter first = pointData.begin();
				first != pointData.end(); ++first) {
			for (PositionElementConstIter second = pointData.begin();
					second != pointData.end(); ++second) {
				if (first == second) {
					continue;
				}

				double slamMapDistance = distance(first->slam, second->slam);
				double realWorldDistance = distance(first->gps, second->gps);

				if (realWorldDistance < MIN_GPS_DISTANCE_M || slamMapDistance < 0.01) {
					// avoid small distances
					continue;
				}

				double scaleEstimate = realWorldDistance / slamMapDistance;

				if(scaleEstimate > 10.) {
					// drop too big estimates
					continue;
				}

				scales.push_back(scaleEstimate);
			}
		};

		if (!scales.empty()) {
			scale = median(scales);
			isValid = true;
			ROS_INFO("Estimated scale by GPS (median from %ld values): %.4f",
					scales.size(), scale);
		} else {
			ROS_INFO("No new scale estimation available.");
		}
	}
}

double SlamScaleManager::distance(const geodesy::UTMPoint &p1,
		const geodesy::UTMPoint &p2) {
	return sqrt(
			pow(p1.easting - p2.easting, 2.)
					+ pow(p1.northing - p2.northing, 2.));
}

double SlamScaleManager::distance(const geometry_msgs::Point &p1,
		const geometry_msgs::Point &p2) {
	return sqrt(pow(p1.x - p2.x, 2.) + pow(p1.y - p2.y, 2.));
}

double SlamScaleManager::median(std::vector<double> &values) {
	std::sort(values.begin(), values.end());
	size_t size = values.size();

	if (size % 2 == 0) {
		return (values.at(size / 2 - 1) + values.at(size / 2))
				/ 2.;
	} else {
		return values.at(values.size() / 2);
	}
}

} /* namespace known_map_localization */
