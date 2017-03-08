/*
 * Localization.cpp
 *
 *  Created on: 19.10.2016
 *      Author: jacob
 */

#include <localization/Localization.h>

#include <geometry_msgs/TransformStamped.h>
#include <geodesy/utm.h>

#include <known_map_localization/GeoPoseStamped.h>
#include <Exception.h>

namespace kml {

Localization::Localization(FilterConstPtr pFilter, SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) :
		pFilter_(pFilter), pSlamScaleManager_(pSlamScaleManager), pKnownMapServer_(pKnownMapServer) {
	ROS_ASSERT(pFilter_);
	ROS_ASSERT(pSlamScaleManager_);
	ROS_ASSERT(pKnownMapServer_);

	ROS_INFO("Localization initialization...");

	ros::NodeHandle nh("~");

	// set up timer
	unsigned int localizationRate = nh.param("localization_rate", 10);
	mTimer_ = nh.createWallTimer(ros::WallDuration(1. / localizationRate), &Localization::tick, this);

	ROS_INFO("    Rate: %d", localizationRate);

	mBaseLinkPublisher_ = nh.advertise<geometry_msgs::TransformStamped>("kml_base_link", 100);
	mGeoPosePublisher_ = nh.advertise<known_map_localization::GeoPoseStamped>("geo_pose", 100);
}

void Localization::localize() {
	tf::Stamped<tf::Pose> robotPose;

	try {
		// get filtered alignment
		Alignment alignment = pFilter_->getAlignment();

		// localize using latest SLAM base link
		robotPose = localizeWithAlignment(alignment);
	} catch(tf::TransformException &e) {
		ROS_DEBUG("Couldn't localize because SLAM base link lookup failed: %s", e.what());
		return;
	} catch(ScaleNotAvailable &e) {
		return;
	} catch(AlignmentNotAvailable &e) {
		return;
	}

	tf::StampedTransform baseLink(robotPose, robotPose.stamp_, "anchor", "kml_base_link");
	geometry_msgs::TransformStamped gBaseLink;
	tf::transformStampedTFToMsg(baseLink, gBaseLink);

	// publish base link (tf + topic)
	mBroadcaster_.sendTransform(baseLink);
	mBaseLinkPublisher_.publish(gBaseLink);

	// publish as geographic pose
	publishGeoPose(baseLink);
}

tf::Stamped<tf::Pose> Localization::localizeWithAlignment(const Alignment &alignment, ros::Time time) const {

	// SLAM map frame -> known map frame
	tf::Transform slamF_to_knownF = alignment.toTfTransform();

	// SLAM map frame -> SLAM base link
        tf::StampedTransform slamF_to_slamBL = 	pSlamScaleManager_->getSlamBaseLinkToMap(time);

	// discard z component
	slamF_to_slamBL.getOrigin().setZ(0);

	tf::Stamped<tf::Pose> pose;
	pose.stamp_ = slamF_to_slamBL.stamp_;
	pose.setIdentity();
	pose *= slamF_to_knownF.inverse();
	pose *= slamF_to_slamBL;
	return pose;
}

void Localization::tick(const ros::WallTimerEvent& event) {
	localize();
}

void Localization::publishGeoPose(const tf::StampedTransform &baseLink) const {
	geographic_msgs::GeoPoseConstPtr anchor = pKnownMapServer_->getAnchor();
	tf::Quaternion anchorRotation;
	tf::quaternionMsgToTF(anchor->orientation, anchorRotation);
	tf::Transform t(anchorRotation);

	// transform base link to align x axis with circles of latitude
	tf::Pose cartesianPose = t * baseLink;

	// get robot position in UTM coordinates
	geodesy::UTMPoint utmAnchor(anchor->position);
	geodesy::UTMPoint utmGeoPosition(
			utmAnchor.easting + cartesianPose.getOrigin().x(),
			utmAnchor.northing + cartesianPose.getOrigin().y(), utmAnchor.zone,
			utmAnchor.band);

	// convert to geographic position and add orientation
	known_map_localization::GeoPoseStamped geoPose;
	geoPose.stamp = baseLink.stamp_;
	geoPose.pose.position = geodesy::toMsg(utmGeoPosition);
	tf::quaternionTFToMsg(cartesianPose.getRotation(), geoPose.pose.orientation);

	mGeoPosePublisher_.publish(geoPose);
}

} /* namespace kml */
