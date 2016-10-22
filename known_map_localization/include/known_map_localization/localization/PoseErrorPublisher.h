/*
 * PoseErrorPublisher.h
 *
 *  Created on: 19.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOCALIZATION_POSEERRORPUBLISHER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOCALIZATION_POSEERRORPUBLISHER_H_

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <logging/DataLogger.h>

namespace kml {

/**
 * # Pose Error Publisher
 *
 * ## Parameters
 * - **compute_pose_error**: Enables or disables computation of pose error. If no ground truth information is available disabling makes sense, since no error can be computed.
 *
 * ## Published Topics
 * - **pose_error**: The position error (difference between estimated and ground truth position). Assuming that the known map anchor lies over the blender origin.
 * - **tf**: The ground truth pose, which has been received is inserted into the tf tree.
 *
 * ## Subscribed Topics
 * - __/pose__: The ground truth pose, e.g. published by the MORSE simulator
 * - **kml_base_link**: The localization base link used to compute the pose error
 */
class PoseErrorPublisher {
public:
	PoseErrorPublisher(DataLoggerPtr pDataLogger);

private:
	/**
	 * Callback for the ground truth topic subscriber.
	 * @param poseMessage The received pose message
	 */
	void receiveGroundTruth(geometry_msgs::PoseStamped poseMessage);

	/**
	 * Callback for the base link topic subscriber.
	 * @param baseLink The received base link message
	 */
	void receiveBaseLink(const geometry_msgs::TransformStamped &baseLinkMsg);

private:
	/// Time of last error computation
	ros::Time mLastTime_;

	/// Receives the ground truth pose over the /pose topic
	ros::Subscriber mGroundTruthSubscriber_;

	/// Receives the localization base link
	ros::Subscriber mBaseLinkSubscriber_;

	/// Publishes the difference between estimated pose and ground truth pose
	ros::Publisher mPoseErrorPublisher_;

	/// Broadcaster for tf transforms
	tf::TransformBroadcaster mBroadcaster_;

	/// Listener for tf transforms
	tf::TransformListener mListener_;

private:
	DataLoggerPtr pDataLogger_;
};

typedef boost::shared_ptr<PoseErrorPublisher> PoseErrorPublisherPtr;
typedef boost::shared_ptr<PoseErrorPublisher const> PoseErrorPublisherConstPtr;

/**
 * Returns the earlier time of the two given.
 * @param t1 First time
 * @param t2 Second time
 * @return The earlier one of the two
 */
ros::Time minimum(const ros::Time &t1, const ros::Time &t2);

/**
 * Computes the absolute distance between two poses, ignoring differences in z direction.
 * @param p1 The first pose
 * @param p2 The second pose
 * @return The distance
 */
float poseToPoseAbsDistance(const tf::Pose &p1, const tf::Pose &p2);

/**
 * Compute the angle (around z axis) in degrees between two orientations.
 * @param q1 The first orientation
 * @param q2 The second orientation
 * @return The angle [degrees]
 */
float orientationToOrientationAngle(const tf::Quaternion &q1, const tf::Quaternion &q2);

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOCALIZATION_POSEERRORPUBLISHER_H_ */
