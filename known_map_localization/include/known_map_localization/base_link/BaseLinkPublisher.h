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

#include <filter/Filter.h>
#include <geographic_msgs/GeoPose.h>

namespace known_map_localization {
namespace base_link {

/**
 * # BaseLinkPublisher
 * Publishes a base link and the map transform on a regular basis using the current filtered alignment.
 *
 * ## Published Topics
 * - **tf**: The transformations topic
 *
 * ## Parameters
 * - **slam_map_scale**: The a priori known scale of the SLAM map, e.g. used with the mapmerge algorithm.
 */
class BaseLinkPublisher {
public:
	/**
	 * Creates the base link publisher.
	 * @param filter The filter where the filtered alignment will be fetched
	 * @param anchor The known map anchor
	 * @param duration The duration between two publications of base links
	 */
	BaseLinkPublisher(filter::FilterConstPtr filter, geographic_msgs::GeoPoseConstPtr anchor, ros::WallDuration duration);

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
	 */
	void updateBaseLink();

	/**
	 * Updates the position based on the anchor and recent base link.
	 */
	void updatePosition();

	/// Holds the filter
	filter::FilterConstPtr filter;

	/// The ROS timer causing regular updates
	ros::WallTimer timer;

	/// Broadcaster for the base link transform
	tf::TransformBroadcaster broadcaster;

	/// Listens to tf messages to get the **ORB_base_link** transformation
	tf::TransformListener listener;

	/// The static SLAM scale
	float slamScale;

	/// The known map anchor
	geographic_msgs::GeoPoseConstPtr anchor;
};

typedef boost::shared_ptr<BaseLinkPublisher> BaseLinkPublisherPtr;
typedef boost::shared_ptr<BaseLinkPublisher const> BaseLinkPublisherConstPtr;
} /* namespace base_link */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_ */
