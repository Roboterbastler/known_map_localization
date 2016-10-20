/*
 * Filter.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <filter/Filter.h>

#include <Exception.h>

namespace kml {

Filter::Filter(SlamScaleManagerPtr pSlamScaleManager, StatusPublisherPtr pStatusPublisher, DataLoggerPtr pDataLogger) :
		mReady_(false), pSlamScaleManager_(pSlamScaleManager), pStatusPublisher_(pStatusPublisher), pDataLogger_(
				pDataLogger) {
	ROS_ASSERT(pSlamScaleManager_);

	ROS_INFO("Filter initialization...");

	ros::NodeHandle nh("~");

	unsigned int updateRate = nh.param("map_transform_rate", 10);
	mTimer_ = nh.createWallTimer(ros::WallDuration(1. / updateRate), &Filter::tick, this);

	ROS_INFO("    Map transform rate: %d", updateRate);
}

Filter::~Filter() {
}

const Alignment& Filter::getAlignment() const {
	if (!mReady_) {
		throw AlignmentNotAvailable("Filtered alignment is not yet available");
	}
	return *pFilteredAlignment_;
}

bool Filter::isAvailable() const {
	return mReady_;
}

void Filter::logAlignment(const Alignment &alignment) {
	if (pDataLogger_) {
		pDataLogger_->logFilter(alignment);
	}
}

void Filter::updateMapTransform() {
	if(mReady_) {
		// publish map transform
		ROS_ERROR("Filter pub map transform frames %s -> %s", StampedAlignment(*pFilteredAlignment_).toTfStampedTransform().frame_id_.c_str(), StampedAlignment(*pFilteredAlignment_).toTfStampedTransform().child_frame_id_.c_str());
		mBroadcaster_.sendTransform(StampedAlignment(*pFilteredAlignment_).toTfStampedTransform());
	}
}

void Filter::tick(const ros::WallTimerEvent& event) {
	updateMapTransform();
}

} /* namespace kml */
