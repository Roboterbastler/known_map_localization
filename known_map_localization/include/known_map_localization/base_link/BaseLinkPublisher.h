/*
 * BaseLinkPublisher.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_

#include <boost/shared_ptr.hpp>

#include <ros/duration.h>
#include <ros/timer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <orb_slam/ORBState.h>

#include <filter/Filter.h>
#include <geographic_msgs/GeoPose.h>

namespace known_map_localization {
namespace base_link {

class BaseLinkPublisher;
typedef boost::shared_ptr<BaseLinkPublisher> BaseLinkPublisherPtr;
typedef boost::shared_ptr<BaseLinkPublisher const> BaseLinkPublisherConstPtr;

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
	static BaseLinkPublisherPtr instance();

	/**
	 * Computes the absolute distance between two poses, ignoring differences in z direction.
	 * @param p1 The first pose
	 * @param p2 The second pose
	 * @return The distance
	 */
	static float poseToPoseAbsDistance(const tf::Pose &p1, const tf::Pose &p2);

protected:
	BaseLinkPublisher();

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
	void receiveGroundTruth(geometry_msgs::PoseStampedConstPtr poseMessage);

private:
	static BaseLinkPublisherPtr _instance;

	/// The ROS timer causing regular updates
	ros::WallTimer timer;

	/// Broadcaster for the base link transform
	tf::TransformBroadcaster broadcaster;

	/// Listens to tf messages to get the **ORB_base_link** transformation
	tf::TransformListener listener;

	/// Receives the ground truth pose over the /pose topic
	ros::Subscriber groundTruthSubscriber;

	/// Publishes a corrected ground truth pose that takes possible offsets
	/// between /world and /ORB_SLAM/World (due to re-localization) into account
	ros::Publisher groundTruthPublisher;

	/// Publishes the difference between estimated pose and ground truth pose
	ros::Publisher poseErrorPublisher;

	/// Publishes the estimated position in geographic coordinates
	ros::Publisher geoPosePublisher;

	/// The ground truth pose received over the /pose topic
	geometry_msgs::PoseStampedConstPtr groundTruth;
};

} /* namespace base_link */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_ */
