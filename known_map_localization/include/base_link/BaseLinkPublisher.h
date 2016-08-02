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

class BaseLinkPublisher {
public:
	BaseLinkPublisher(filter::FilterConstPtr filter, ros::Duration duration);

private:
	void updateBaseLink(const ros::TimerEvent& event);

	filter::FilterConstPtr filter;

	ros::WallTimer timer;

	tf::TransformBroadcaster broadcaster;
	tf::TransformListener listener;
};

typedef boost::shared_ptr<BaseLinkPublisher> BaseLinkPublisherPtr;
typedef boost::shared_ptr<BaseLinkPublisher const> BaseLinkPublisherConstPtr;
} /* namespace base_link */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_BASE_LINK_BASELINKPUBLISHER_H_ */
