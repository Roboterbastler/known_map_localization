/*
 * SlamScaleManager.cpp
 *
 *  Created on: 27.08.2016
 *      Author: jacob
 */

#include <SlamScaleManager.h>

#include <numeric>

#include <ros/ros.h>
#include <geodesy/wgs84.h>

#include <Exception.h>
#include <Utils.h>

#define MAX_POSITION_CACHE_SIZE 100
#define MIN_GPS_DISTANCE_M 0.5

namespace kml {

SlamScaleManager::SlamScaleManager(GpsManagerConstPtr pGpsManager,
		DataLoggerPtr pDataLogger) :
		mMode_(determineMode()), mIsValid_(false), mScale_(1.0), pDataLogger_(
				pDataLogger), pGpsManager_(pGpsManager) {
	ROS_ASSERT(pGpsManager_);

	ros::NodeHandle nh("~");

	ROS_INFO("SLAM scale manager initialization...");

	switch (mMode_) {
	case PARAMETER:
		ROS_INFO("    Mode: PARAMETER");
		if (nh.getParam("slam_map_scale", mScale_)) {
			ROS_INFO("    Scale: %f", mScale_);
			mIsValid_ = true;
			if (pDataLogger_) {
				pDataLogger_->logScale(mScale_, mMode_);
			}
		} else {
			ROS_FATAL("No SLAM scale parameter found.");
			ROS_BREAK();
			return;
		}
		break;
	case ALIGNMENT:
		ROS_INFO("    Mode: ALIGNMENT");
		break;
	case GPS:
		ROS_INFO("    Mode: GPS");
		mGpsHintsUpdatedSubscriber_ = nh.subscribe("gps_hints_updated", 1,
				&SlamScaleManager::estimateScale, this);
		break;
	default:
		ROS_FATAL("    Illegal or not specified SLAM scale manager mode.");
		ROS_BREAK();
		return;
	}
}

SlamScaleMode SlamScaleManager::getMode() const {
	return mMode_;
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
	if (mIsValid_) {
		return mScale_;
	} else {
		throw ScaleNotAvailable("Scale estimate not available");
	}
}

void SlamScaleManager::updateSlamScale(float scale) {
	if (mMode_ == ALIGNMENT) {
		this->mScale_ = scale;
		mIsValid_ = true;
		if (pDataLogger_) {
			pDataLogger_->logScale(scale, mMode_);
		}
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

void SlamScaleManager::estimateScale(const std_msgs::Empty &signal) {
	const GpsHintVect &gpsHints = pGpsManager_->getGpsHints();

	ROS_INFO("Estimate new scale using GPS hints...");

	if (gpsHints.size() >= 2) {
		std::vector<double> scales;

		for (GpsHintVect::const_iterator first = gpsHints.begin();
				first != gpsHints.end(); ++first) {
			for (GpsHintVect::const_iterator second = gpsHints.begin();
					second != gpsHints.end(); ++second) {
				if (first == second) {
					continue;
				}

				geometry_msgs::Point firstPoint, secondPoint;
				tf::pointTFToMsg(first->baseLink.getOrigin(), firstPoint);
				tf::pointTFToMsg(second->baseLink.getOrigin(), secondPoint);

				double slamMapDistance = kml::distance(firstPoint, secondPoint);
				double realWorldDistance = kml::distance(first->gpsPosition,
						second->gpsPosition);

				if (realWorldDistance < MIN_GPS_DISTANCE_M
						|| slamMapDistance < 0.01) {
					// avoid small distances
					continue;
				}

				double scaleEstimate = realWorldDistance / slamMapDistance;

				scales.push_back(scaleEstimate);
			}
		};

		if (!scales.empty()) {
			mScale_ = median(scales);
			mIsValid_ = true;
			if (pDataLogger_) {
				pDataLogger_->logScale(mScale_, mMode_);
			}
			ROS_INFO(
					"  - Estimated scale by GPS (median from %ld values): %.4f",
					scales.size(), mScale_);
		} else {
			ROS_INFO("  - No new scale estimation available.");
		}
	}
}

} /* namespace kml */
