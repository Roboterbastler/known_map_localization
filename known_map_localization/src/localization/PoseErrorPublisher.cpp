/*
 * PoseErrorPublisher.cpp
 *
 *  Created on: 19.10.2016
 *      Author: jacob
 */

#include <localization/PoseErrorPublisher.h>

#include <Utils.h>

namespace kml {

PoseErrorPublisher::PoseErrorPublisher(DataLoggerPtr pDataLogger) : mLastTime_(0), pDataLogger_(pDataLogger) {
	ROS_ASSERT(pDataLogger_);

	ROS_INFO("Pose Error Publisher initialization...");

	ros::NodeHandle nh("~");

	bool enabled = nh.param("compute_pose_error", true);

	ROS_INFO("    Enabled: %s", enabled ? "True" : "False");

	if(enabled) {
		mGroundTruthSubscriber_ = nh.subscribe("/pose", 10, &PoseErrorPublisher::receiveGroundTruth, this);
		mBaseLinkSubscriber_ = nh.subscribe("kml_base_link", 10, &PoseErrorPublisher::receiveBaseLink, this);
		mPoseErrorPublisher_ = nh.advertise<known_map_localization::PoseError>("pose_error", 1000);
	}
}

void PoseErrorPublisher::receiveGroundTruth(geometry_msgs::PoseStamped poseMessage) {
	// insert ground truth pose into tf tree
	tf::Stamped<tf::Pose> groundTruthPose;
	tf::poseStampedMsgToTF(poseMessage, groundTruthPose);
	tf::StampedTransform groundTruth(groundTruthPose, groundTruthPose.stamp_, "anchor", "ground_truth");
	mBroadcaster_.sendTransform(groundTruth);
}

void PoseErrorPublisher::receiveBaseLink(const geometry_msgs::TransformStamped &baseLinkMsg) {
	// ground truth pose in known map anchor frame
	tf::StampedTransform groundTruthPose;
	try {
		mListener_.lookupTransform("anchor", "ground_truth", ros::Time(0), groundTruthPose);
	} catch(tf::TransformException &e) {
		ROS_DEBUG("Pose error not updated: Ground truth pose not available: %s", e.what());
		return;
	}

	// get latest common time of ground truth pose and
	ros::Time latestCommonTime = minimum(groundTruthPose.stamp_, baseLinkMsg.header.stamp);

	if(latestCommonTime <= mLastTime_) {
		// no new error information available
		return;
	}

	// estimated pose in known map anchor frame
	tf::StampedTransform baseLink;
	try {
		mListener_.lookupTransform("anchor", "ground_truth", latestCommonTime, groundTruthPose);
		mListener_.lookupTransform("anchor", "kml_base_link", latestCommonTime, baseLink);
	} catch(tf::TransformException &e) {
		ROS_DEBUG("Pose error not updated: %s", e.what());
		return;
	}

	mLastTime_ = latestCommonTime;

	// publish pose error
	known_map_localization::PoseError poseError;
	poseError.translational_error = poseToPoseAbsDistance(baseLink, groundTruthPose);
	poseError.rotational_error = orientationToOrientationAngle(baseLink.getRotation(), groundTruthPose.getRotation());
	poseError.header.stamp = latestCommonTime;
	mPoseErrorPublisher_.publish(poseError);

	if (pDataLogger_) {
		pDataLogger_->logError(poseError);
	}
}

ros::Time minimum(const ros::Time &t1, const ros::Time &t2) {
	if(t1 < t2) {
		return t1;
	} else {
		return t2;
	}
}

float poseToPoseAbsDistance(const tf::Pose &p1, const tf::Pose &p2) {
	float dx = p2.getOrigin().x() - p1.getOrigin().x();
	float dy = p2.getOrigin().y() - p1.getOrigin().y();
	return sqrt(pow(dx, 2) + pow(dy, 2));
}

float orientationToOrientationAngle(const tf::Quaternion &q1, const tf::Quaternion &q2) {
	// discard other angles
	tf::Quaternion rot1 = tf::createQuaternionFromYaw(tf::getYaw(q1));
	tf::Quaternion rot2 = tf::createQuaternionFromYaw(tf::getYaw(q2));
	return radToDeg(rot1.angle(rot2) * 2.);
}

} /* namespace kml */
