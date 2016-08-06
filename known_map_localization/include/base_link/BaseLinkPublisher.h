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

#include "filter/Filter.h"

namespace known_map_localization {
namespace base_link {

/**
 * # BaseLinkPublisher
 * Publishes a base link on a regular basis using the current filtered alignment.
 *
 * ## Published Topics
 * - __/known_map_localization/base_link__: The current base link.
 */
class BaseLinkPublisher {
public:
	/**
	 * Creates the base link publisher.
	 * @param filter The filter where the filtered alignment will be fetched
	 * @param duration The duration between two publications of base links
	 */
	BaseLinkPublisher(filter::FilterConstPtr filter, ros::WallDuration duration);

private:
	/**
	 * Is called by a ROS WallTimer to update and publish the base link.
	 * @param event The WallTimer event
	 */
	void updateBaseLink(const ros::WallTimerEvent& event);

	/// Holds the filter
	filter::FilterConstPtr filter;

	/// The ROS timer causing regular updates
	ros::WallTimer timer;

	/// Broadcaster for the base link transform
	tf::TransformBroadcaster broadcaster;

	/// Listens to tf messages to get the **ORB_base_link** transformation
	tf::TransformListener listener;
};

typedef boost::shared_ptr<BaseLinkPublisher> BaseLinkPublisherPtr;
typedef boost::shared_ptr<BaseLinkPublisher const> BaseLinkPublisherConstPtr;
} /* namespace base_link */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_ */
