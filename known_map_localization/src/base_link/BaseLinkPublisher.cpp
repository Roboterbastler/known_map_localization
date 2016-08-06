/*
 * BaseLinkPublisher.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <base_link/BaseLinkPublisher.h>
#include <Exception.h>

namespace known_map_localization {
namespace base_link {

BaseLinkPublisher::BaseLinkPublisher(filter::FilterConstPtr filter, ros::WallDuration duration) : filter(filter) {
	if(ros::isInitialized()) {
		ros::NodeHandle nh;
		timer = nh.createWallTimer(duration, &BaseLinkPublisher::updateBaseLink, this);
	}
}

void BaseLinkPublisher::updateBaseLink(const ros::WallTimerEvent& event) {
	// request /orb_slam/map to /orb_slam/base_link
	// scale it
	// transform.inverse() + scaled ORB base link

	// get filtered alignment
	alignment::Alignment alignment;
	try {
		alignment = filter->getAlignment();
	} catch(AlignmentNotAvailable &e) {
		return;
	}

	tf::StampedTransform slamMapFrame_to_slamBaseLink;
	tf::Transform slamMapFrame_to_knownMapFrame = alignment.toTfTransform();

	try {
		listener.lookupTransform("orb_slam/map", "ORB_base_link", ros::Time(0), slamMapFrame_to_slamBaseLink);
	} catch(tf::TransformException &e) {
		ROS_WARN("Required tf data not available, base_link not published: %s", e.what());
		return;
	}

	// apply scale
	slamMapFrame_to_slamBaseLink.getOrigin() *= alignment.scale;

	tf::Transform kmlBaseLink;
	kmlBaseLink.setIdentity();
	kmlBaseLink *= slamMapFrame_to_knownMapFrame.inverse();
	kmlBaseLink *= slamMapFrame_to_slamBaseLink;

	broadcaster.sendTransform(tf::StampedTransform(kmlBaseLink, ros::Time::now(), "/known_map_localization/anchor", "/known_map_localization/base_link"));
}

} /* namespace base_link */
} /* namespace known_map_localization */
