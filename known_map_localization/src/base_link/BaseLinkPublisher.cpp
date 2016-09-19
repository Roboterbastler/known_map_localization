/*
 * BaseLinkPublisher.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <base_link/BaseLinkPublisher.h>
#include <Exception.h>
#include <SlamScaleManager.h>
#include <known_map_server/KnownMapServer.h>
#include <known_map_localization/PoseError.h>

namespace known_map_localization {
namespace base_link {

BaseLinkPublisherPtr BaseLinkPublisher::_instance;

BaseLinkPublisher::BaseLinkPublisher() {
	ROS_INFO("Base link publisher initialization...");

	ros::WallDuration interval(0.2);

	ROS_INFO("    Interval: %.2f", interval.toSec());

	if (ros::isInitialized()) {
		ros::NodeHandle nh("~");
		timer = nh.createWallTimer(interval, &BaseLinkPublisher::update, this);

		groundTruthSubscriber = nh.subscribe("/pose", 10, &BaseLinkPublisher::receiveGroundTruth, this);
		groundTruthPublisher = nh.advertise<geometry_msgs::PoseStamped>("ground_truth", 1000);
		geoPosePublisher = nh.advertise<geographic_msgs::GeoPose>("geo_pose", 1000);
		poseErrorPublisher = nh.advertise<PoseError>("pose_error", 1000);
	}
}

BaseLinkPublisherPtr BaseLinkPublisher::instance() {
	if(!_instance) {
		_instance = BaseLinkPublisherPtr(new BaseLinkPublisher());
	}
	return _instance;
}

void BaseLinkPublisher::update(const ros::WallTimerEvent& event) {
	// update map-to-map transform
	updateMapTransform();

	// update base link
	tf::StampedTransform baseLink;
	if(updateBaseLink(baseLink)) {
		// update positions
		updatePositionError(baseLink);
		updateGeoPose(known_map_server::KnownMapServer::instance()->getAnchor(), baseLink);
	}
}

void BaseLinkPublisher::updateMapTransform() {
	// get filtered alignment
	alignment::Alignment alignment;
	try {
		alignment = filter::Filter::instance()->getAlignment();

		// publish map transform
		broadcaster.sendTransform(
				alignment::StampedAlignment(alignment).toTfStampedTransform());
	} catch (AlignmentNotAvailable &e) {
	}
}

bool BaseLinkPublisher::updateBaseLink(tf::StampedTransform &out) {
	try {
		// get filtered alignment
		const alignment::Alignment &alignment = filter::Filter::instance()->getAlignment();

		// get pose for alignment
		tf::Stamped<tf::Pose> copterPose = getPoseForAlignment(alignment);

		// create base link
		out = tf::StampedTransform(copterPose, copterPose.stamp_,
					"/known_map_localization/anchor",
					"/known_map_localization/base_link");

		// publish base link
		broadcaster.sendTransform(out);
		return true;
	} catch (AlignmentNotAvailable &e) {
		return false;
	} catch (tf::TransformException &e) {
		ROS_WARN_THROTTLE(1, "Getting pose for alignment failed: %s", e.what());
		return false;
	} catch(ScaleNotAvailable &e) {
		return false;
	}
}

tf::Stamped<tf::Pose> BaseLinkPublisher::getPoseForAlignment(const alignment::Alignment &alignment) {
	tf::StampedTransform slamMapFrame_to_slamBaseLink;
	tf::Transform slamMapFrame_to_knownMapFrame = alignment.toTfTransform();

	listener.lookupTransform("orb_slam/map", "ORB_base_link", ros::Time(0), slamMapFrame_to_slamBaseLink);
	
	// convert pose to real world scale
	slamMapFrame_to_slamBaseLink = SlamScaleManager::instance()->convertTransform(slamMapFrame_to_slamBaseLink);

	tf::Stamped<tf::Pose> pose;
	pose.stamp_ = slamMapFrame_to_slamBaseLink.stamp_;
	pose.setIdentity();
	pose *= slamMapFrame_to_knownMapFrame.inverse();
	pose *= slamMapFrame_to_slamBaseLink;
	return pose;
}

void BaseLinkPublisher::updatePositionError(const tf::StampedTransform &baseLink) {
	if(!groundTruth) {
		ROS_DEBUG_THROTTLE(2, "Ground truth pose not available. Pose error not published.");
		return;
	}

	try {
		// estimated pose in known map anchor frame
		tf::Stamped<tf::Pose> estimatedPose(baseLink, baseLink.stamp_, baseLink.frame_id_);

		// ground truth pose in known map anchor frame
		tf::Stamped<tf::Pose> groundTruthPose;
		tf::poseStampedMsgToTF(*groundTruth, groundTruthPose);

		// publish pose error
		PoseError poseError;
		poseError.translational_error = poseToPoseAbsDistance(estimatedPose, groundTruthPose);
		poseError.rotational_error = 2. * estimatedPose.getRotation().angle(groundTruthPose.getRotation());
		poseError.header.stamp = baseLink.stamp_;
		poseErrorPublisher.publish(poseError);
	} catch(tf::TransformException &e) {
		ROS_WARN_THROTTLE(2, "Could not get estimated pose or ground truth pose: %s", e.what());
	}
}

void BaseLinkPublisher::updateGeoPose(geographic_msgs::GeoPoseConstPtr anchor, const tf::StampedTransform &baseLink) {
	tf::Quaternion anchorRotation;
	tf::quaternionMsgToTF(anchor->orientation, anchorRotation);
	tf::Transform t(anchorRotation.inverse());

	// transform base link to align x axis with circles of latitude
	tf::Pose cartesianPose = t * baseLink;

	// use the quick and dirty estimate that 111,111 meters (111.111 km) in the y direction is 1 degree (of latitude)
	// and 111,111 * cos(latitude) meters in the x direction is 1 degree (of longitude).
	geographic_msgs::GeoPose geoPose;
	tf::quaternionTFToMsg(cartesianPose.getRotation(), geoPose.orientation);
	geoPose.position.altitude = anchor->position.altitude + cartesianPose.getOrigin().getZ();
	geoPose.position.latitude = anchor->position.latitude + (cartesianPose.getOrigin().getY() / 111111.);
	geoPose.position.longitude = anchor->position.longitude + (cartesianPose.getOrigin().getX() / (111111. * cos(geoPose.position.latitude)));

	geoPosePublisher.publish(geoPose);
}

void BaseLinkPublisher::receiveGroundTruth(geometry_msgs::PoseStampedConstPtr poseMessage) {
	groundTruth = poseMessage;
}

float BaseLinkPublisher::poseToPoseAbsDistance(const tf::Pose &p1, const tf::Pose &p2) {
	float dx = p2.getOrigin().x() - p1.getOrigin().x();
	float dy = p2.getOrigin().y() - p1.getOrigin().y();
	return sqrt(pow(dx, 2) + pow(dy, 2));
}

} /* namespace base_link */
} /* namespace known_map_localization */
