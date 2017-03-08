/*
 * GpsManager.cpp
 *
 *  Created on: 16.09.2016
 *      Author: jacob
 */

#include <gps/GpsManager.h>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <visualization_msgs/Marker.h>

#include <SlamScaleManager.h>

#include <Exception.h>

#define MAX_HINT_VECTOR_SIZE 100
#define MAX_FIX_QUEUE_SIZE 100

namespace kml {

GpsManager::GpsManager(KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerConstPtr pSlamScaleManager) :
		pKnownMapServer_(pKnownMapServer), pSlamScaleManager_(pSlamScaleManager) {
	ROS_ASSERT(pKnownMapServer_);

	ROS_INFO("GPS manager initialization...");

	ros::WallDuration interval(1.0);
	ROS_INFO("    Interval: %.2f", interval.toSec());

	ros::NodeHandle nh("~");

	mGpsPositionSubscriber_ = nh.subscribe("/gps_fix", 1000,
			&GpsManager::receiveGpsFix, this);
	mTimer_ = nh.createWallTimer(interval, &GpsManager::updateGpsHints, this);
	mGpsMarkerPublisher_ = nh.advertise<visualization_msgs::Marker>(
			"gps_position_marker", 1);
	mGpsHintsUpdatedPublisher_ = nh.advertise<std_msgs::Empty>(
			"gps_hints_updated", 1);
}

void GpsManager::receiveGpsFix(const sensor_msgs::NavSatFix &fix) {
	if (fix.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
		// GPS fix is invalid
		return;
	}

	try {
		GpsPosition position;
		position.gpsPosition = convertGPSPositionToAnchorFrame(fix,
				*(pKnownMapServer_->getAnchor()));
		position.stamp = fix.header.stamp;
		position.gpsFix = fix;

		mPositions_.push_back(position);

		// remove front if queue too full
		if (mPositions_.size() > MAX_FIX_QUEUE_SIZE) {
			mPositions_.erase(mPositions_.begin());
		}

		publishGpsPositionMarker();

		ROS_DEBUG("Received new GPS fix. Queue size: %ld", mPositions_.size());
	} catch (DifferentUTMGridZones &e) {
		ROS_WARN("GPS Manager unsupported fix: %s", e.what());
	}
}

void GpsManager::updateGpsHints(const ros::WallTimerEvent& event) {
	bool modified = false;

	for (GpsPositionVect::iterator pos = mPositions_.begin();
			pos != mPositions_.end();) {
//		if(hintIsOutdated(*hint)) {
//			// remove GPS fix from queue, since there is no chance anymore
//			// to get a SLAM base link for this time
//			hint = hintQueue.erase(hint);
//			continue;
//		}

		try {
			GpsHint hint;
			hint.stamp = pos->stamp;
			hint.gpsFix = pos->gpsFix;
			hint.gpsPosition = pos->gpsPosition;
			hint.baseLink = pSlamScaleManager_->getSlamBaseLinkToMap(hint.stamp);

			// add new key point
			mHints_.push_back(hint);
			modified = true;

			// remove GPS fix from queue
			pos = mPositions_.erase(pos);
		} catch (tf::TransformException &e) {
			ROS_DEBUG("SLAM base link not found for GPS hint: %s", e.what());
			++pos;
		}
	}

	// remove front if queue too full
	while (mHints_.size() > MAX_HINT_VECTOR_SIZE) {
		mHints_.erase(mHints_.begin());
		modified = true;
	}

	publishGpsPositionMarker();
	publishGpsHintMarker();

	if (modified) {
		mGpsHintsUpdatedPublisher_.publish(std_msgs::Empty());
	}

	ROS_DEBUG("Key points: %ld", mHints_.size());
}

geometry_msgs::Point GpsManager::convertGPSPositionToAnchorFrame(
		const sensor_msgs::NavSatFix &gpsFix,
		const geographic_msgs::GeoPose &anchor) {
	geodesy::UTMPoint utmFix(geodesy::toMsg(gpsFix));
	geodesy::UTMPoint utmAnchor(anchor.position);

	if (!geodesy::sameGridZone(utmFix, utmAnchor)) {
		// for simplicity assume region of operation does not spread over multiple UTM grid zones
		// ignore other GPS fixes
		std::stringstream errorMessage;
		errorMessage << "GPS fix lies in a different UTM grid zone ("
				<< int(utmFix.zone) << utmFix.band
				<< ") than the known map anchor (" << int(utmAnchor.zone)
				<< utmAnchor.band << ")";
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

const GpsHintVect& GpsManager::getGpsHints() const {
	return mHints_;
}

void GpsManager::publishGpsHintMarker() {
	visualization_msgs::Marker hintMarker;
	hintMarker.header.frame_id =
			pKnownMapServer_->getKnownMap()->header.frame_id;
	hintMarker.header.stamp = ros::Time::now();
	hintMarker.ns = "GPS-Positions";
	hintMarker.id = 1;
	hintMarker.frame_locked = true;
	hintMarker.type = visualization_msgs::Marker::POINTS;
	hintMarker.action = visualization_msgs::Marker::ADD;
	hintMarker.color.g = 1.0;
	hintMarker.color.a = 1.0;
	float pointSize = 0.2;
	hintMarker.scale.x = pointSize;
	hintMarker.scale.y = pointSize;
	hintMarker.pose.orientation.w = 1.0;

	for (GpsHintVect::const_iterator hint = mHints_.begin();
			hint != mHints_.end(); ++hint) {
		hintMarker.points.push_back(hint->gpsPosition);
	}

	mGpsMarkerPublisher_.publish(hintMarker);
}

void GpsManager::publishGpsPositionMarker() {
	if (mGpsMarkerPublisher_.getNumSubscribers() > 0) {
		visualization_msgs::Marker gpsPositionMarker;
		gpsPositionMarker.header.frame_id =
				pKnownMapServer_->getKnownMap()->header.frame_id;
		gpsPositionMarker.header.stamp = ros::Time::now();
		gpsPositionMarker.ns = "GPS-Fixes";
		gpsPositionMarker.id = 0;
		gpsPositionMarker.frame_locked = true;
		gpsPositionMarker.type = visualization_msgs::Marker::POINTS;
		gpsPositionMarker.action = visualization_msgs::Marker::ADD;
		gpsPositionMarker.color.b = 1.0;
		gpsPositionMarker.color.a = 1.0;
		float pointSize = 0.2;
		gpsPositionMarker.scale.x = pointSize;
		gpsPositionMarker.scale.y = pointSize;
		gpsPositionMarker.pose.orientation.w = 1.0;
		for (GpsPositionVect::const_iterator pos = mPositions_.begin();
				pos != mPositions_.end(); ++pos) {
			gpsPositionMarker.points.push_back(pos->gpsPosition);
		}
		mGpsMarkerPublisher_.publish(gpsPositionMarker);
	}
}

bool GpsManager::gpsPositionIsOutdated(const GpsPosition &pos) {
	ros::Duration age = ros::Time::now() - pos.stamp;
	return age.toSec() > mListener_.getCacheLength().toSec();
}

} /* namespace kml */
