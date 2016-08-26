/*
 * BaseLinkPublisher.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <geometry_msgs/PoseStamped.h>

#include <base_link/BaseLinkPublisher.h>
#include <Exception.h>

namespace known_map_localization {
namespace base_link {

BaseLinkPublisher::BaseLinkPublisher(filter::FilterConstPtr filter,
		geographic_msgs::GeoPoseConstPtr anchor, ros::WallDuration duration) :
		filter(filter),
		anchor(anchor),
		slamState(-1) {
	assert(filter);
	assert(anchor);
	orbMapToScene.setIdentity();

	if (ros::isInitialized()) {
		ros::NodeHandle nh("~");
		timer = nh.createWallTimer(duration, &BaseLinkPublisher::update, this);

		// get static SLAM map scale if available
		slamScale = nh.param("slam_map_scale", 1.0);

		groundTruthSubscriber = nh.subscribe("/pose", 10, &BaseLinkPublisher::receiveGroundTruth, this);
		groundTruthPublisher = nh.advertise<geometry_msgs::PoseStamped>("ground_truth", 1000);
		slamStateSubscriber = nh.subscribe("/orb_slam/state", 1000, &BaseLinkPublisher::receiveSlamState, this);
	}
}

void BaseLinkPublisher::update(const ros::WallTimerEvent& event) {
	// update map-to-map transform
	updateMapTransform();

	// update base link
	tf::StampedTransform baseLink;
	if(updateBaseLink(baseLink)) {
		// update position
		updatePosition(baseLink);
	}
}

void BaseLinkPublisher::updateMapTransform() {
	// get filtered alignment
	alignment::Alignment alignment;
	try {
		alignment = filter->getAlignment();

		// publish map transform
		broadcaster.sendTransform(
				alignment::StampedAlignment(alignment).toTfStampedTransform());
	} catch (AlignmentNotAvailable &e) {
	}
}

bool BaseLinkPublisher::updateBaseLink(tf::StampedTransform &out) {
	// request /orb_slam/map to /orb_slam/base_link
	// scale it
	// transform.inverse() + scaled ORB base link

	// get filtered alignment
	alignment::Alignment alignment;
	try {
		alignment = filter->getAlignment();
	} catch (AlignmentNotAvailable &e) {
		return false;
	}

	tf::StampedTransform slamMapFrame_to_slamBaseLink;
	tf::Transform slamMapFrame_to_knownMapFrame = alignment.toTfTransform();

	try {
		listener.lookupTransform("orb_slam/map", "ORB_base_link", ros::Time(0),
				slamMapFrame_to_slamBaseLink);
	} catch (tf::TransformException &e) {
		ROS_WARN_THROTTLE(1, "Required tf data not available, base_link not published: %s", e.what());
		return false;
	}

	// apply scales
	slamMapFrame_to_slamBaseLink.getOrigin() *= alignment.scale / slamScale;

	tf::Transform kmlBaseLink;
	kmlBaseLink.setIdentity();
	kmlBaseLink *= slamMapFrame_to_knownMapFrame.inverse();
	kmlBaseLink *= slamMapFrame_to_slamBaseLink;

	out = tf::StampedTransform(kmlBaseLink, ros::Time::now(),
			"/known_map_localization/anchor",
			"/known_map_localization/base_link");

	broadcaster.sendTransform(out);
	return true;
}

void BaseLinkPublisher::updatePosition(const tf::StampedTransform &baseLink) {
	if(!groundTruth) {
		ROS_DEBUG_THROTTLE(2, "Ground truth pose not available. Pose not published.");
		return;
	}

	tf::StampedTransform baseLinkSlamWorld;
	try {
		listener.lookupTransform("/orb_slam/map", "/known_map_localization/base_link", ros::Time(0), baseLinkSlamWorld);
	} catch (tf::TransformException &e) {
		ROS_WARN_THROTTLE(2, "Transformation of base link pose failed: %s", e.what());
		return;
	}

	// the estimated pose
	tf::Stamped<tf::Pose> baseLinkPose(baseLinkSlamWorld, baseLinkSlamWorld.stamp_, baseLinkSlamWorld.frame_id_);

	// the ground truth pose in the "/blender_scene" frame
	tf::Stamped<tf::Pose> groundTruthPose;
	tf::poseStampedMsgToTF(*groundTruth, groundTruthPose);

	// convert pose into "/orb_slam/map" frame
	groundTruthPose.setData(orbMapToScene * groundTruthPose);
	groundTruthPose.frame_id_ = "/orb_slam/map";

	// publish ground truth pose
	geometry_msgs::PoseStamped groundTruthPoseMsg;
	tf::poseStampedTFToMsg(groundTruthPose, groundTruthPoseMsg);
	groundTruthPublisher.publish(groundTruthPoseMsg);

	float absPoseError = poseToPoseAbsDistance(baseLinkPose, groundTruthPose);
}

void BaseLinkPublisher::receiveGroundTruth(geometry_msgs::PoseStampedConstPtr poseMessage) {
	groundTruth = poseMessage;
}

void BaseLinkPublisher::receiveSlamState(orb_slam::ORBState stateMessage) {
	// detect "initialization -> tracking" transition
	if(slamState == 2 && stateMessage.state == 3) {
		// store /world to /orb_slam/map
		try {
			listener.lookupTransform("/orb_slam/map", "/blender_scene", ros::Time(0), orbMapToScene);
			ROS_INFO("START OF TRACKING detected. Stored /orb_slam/map to /world transformation: x=%f y=%f z=%f", orbMapToScene.getOrigin().x(), orbMapToScene.getOrigin().y(), orbMapToScene.getOrigin().z());
		} catch(tf::TransformException &e) {
			ROS_WARN("Could not store /orb_slam/map to /world transformation: %s", e.what());
			return;
		}
	}

	slamState = stateMessage.state;
}

float BaseLinkPublisher::poseToPoseAbsDistance(const tf::Pose &p1, const tf::Pose &p2) {
	float dx = p2.getOrigin().x() - p1.getOrigin().x();
	float dy = p2.getOrigin().y() - p1.getOrigin().y();
	float dz = p2.getOrigin().z() - p1.getOrigin().z();
	return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
}

} /* namespace base_link */
} /* namespace known_map_localization */
