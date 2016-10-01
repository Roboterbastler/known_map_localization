/*
 * BaseLinkPublisher.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_

#include <boost/shared_ptr.hpp>

#include <ros/timer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geographic_msgs/GeoPose.h>

#include <alignment/Alignment.h>
#include <logging/DataLogger.h>
#include <known_map_server/KnownMapServer.h>
#include <SlamScaleManager.h>
#include <filter/Filter.h>

namespace kml {

/**
 * # BaseLinkPublisher
 * Publishes a base link and the map transform on a regular basis using the current filtered alignment.
 *
 * ## Published Topics
 * - **tf**: The transformations topic
 * - **pose_error**: The position error (difference between estimated and ground truth position). Assuming that the known map anchor lies over the blender origin.
 * - **geo_pose**: The current estimated geographic pose using the WGS 84 reference ellipsoid
 *
 * ## Parameters
 * - **slam_map_scale**: The a priori known scale of the SLAM map, e.g. used with the mapmerge algorithm.
 */
class BaseLinkPublisher {
public:
	BaseLinkPublisher(KnownMapServerConstPtr pKnownMapServer, FilterConstPtr pFilter, SlamScaleManagerConstPtr pSlamScaleManager, DataLoggerPtr pDataLogger = DataLoggerPtr());

	/**
	 * Computes the absolute distance between two poses, ignoring differences in z direction.
	 * @param p1 The first pose
	 * @param p2 The second pose
	 * @return The distance
	 */
	static float poseToPoseAbsDistance(const tf::Pose &p1, const tf::Pose &p2);

	/**
	 * Compute the angle (around z axis) in degrees between two orientations.
	 * @param q1 The first orientation
	 * @param q2 The second orientation
	 * @return The angle [degrees]
	 */
	static float orientationToOrientationAngle(const tf::Quaternion &q1, const tf::Quaternion &q2);

	/**
	 * Computes the copter pose based on the given alignment and current SLAM base link.
	 * The pose is given in the known map anchor frame.
	 * @param alignment The alignment used to compute the pose
	 * @return The copter pose, the stamp indicates the time stamp of the SLAM base link used
	 * @throws tf::TransformException If tf lookup of SLAM base link fails.
	 * @throws ScaleNotAvailable If no scale is available.
	 */
	tf::Stamped<tf::Pose> getPoseForAlignment(const Alignment &alignment);

private:
	/**
	 * Is called by a ROS WallTimer to update and publish the base link.
	 * @param event The WallTimer event
	 */
	void update(const ros::WallTimerEvent& event);

	/**
	 * Updates the tf transform from the SLAM map frame to the known map frame.
	 */
	void updateMapTransform();

	/**
	 * Update and publish the base link.
	 * @param out Outputs the new base link
	 * @return Returns whether a new base link was generated
	 */
	bool updateBaseLink(tf::StampedTransform &out);

	/**
	 * Updates the position error based on the anchor and recent base link.
	 * @param baseLink The latest base link used to update the position
	 */
	void updatePositionError(const tf::StampedTransform &baseLink);

	/**
	 * Updates the geo pose based on the geographic pose of the anchor and the current base link.
	 * @param anchor The known map anchor
	 * @param baseLink The current base link
	 */
	void updateGeoPose(geographic_msgs::GeoPoseConstPtr anchor, const tf::StampedTransform &baseLink);

	/**
	 * Callback for the ground truth topic subscriber.
	 * @param poseMessage The received pose message
	 */
	void receiveGroundTruth(const geometry_msgs::PoseStamped &poseMessage);

private:

	/// The ROS timer causing regular updates
	ros::WallTimer mTimer_;

	/// Broadcaster for tf transforms
	tf::TransformBroadcaster mBroadcaster_;

	/// Listens to tf messages to get the **ORB_base_link** transformation
	tf::TransformListener mListener_;

	/// Receives the ground truth pose over the /pose topic
	ros::Subscriber mGroundTruthSubscriber_;

	/// Publishes the difference between estimated pose and ground truth pose
	ros::Publisher mPoseErrorPublisher_;

	/// Publishes the estimated position in geographic coordinates
	ros::Publisher mGeoPosePublisher_;

private:
	KnownMapServerConstPtr pKnownMapServer_;
	FilterConstPtr pFilter_;
	SlamScaleManagerConstPtr pSlamScaleManager_;
	DataLoggerPtr pDataLogger_;
};

typedef boost::shared_ptr<BaseLinkPublisher> BaseLinkPublisherPtr;
typedef boost::shared_ptr<BaseLinkPublisher const> BaseLinkPublisherConstPtr;

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_ */
