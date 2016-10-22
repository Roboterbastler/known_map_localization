/*
 * Status.cpp
 *
 *  Created on: 17.10.2016
 *      Author: jacob
 */

#include <StatusPublisher.h>

namespace kml {

StatusPublisher::StatusPublisher(DataLoggerPtr pDataLogger, float rate) :
		mStatus_(STATUS_INIT), mNSupportingGpsPos_(0), pDataLogger_(pDataLogger) {
	ROS_ASSERT(pDataLogger_);

	ros::NodeHandle nh("~");
	mStatusPublisher_ = nh.advertise<known_map_localization::Status>("status", 1);
	mSlamStatusSubscriber_ = nh.subscribe("/orb_slam/state", 10, &StatusPublisher::receiveSlamState, this);
	mTimer_ = nh.createWallTimer(ros::WallDuration(1. / rate), &StatusPublisher::tick, this);
}

void StatusPublisher::setStatus(Status s, unsigned int nSupportingGpsPos) {
	mStatus_ = s;
	mNSupportingGpsPos_ = nSupportingGpsPos;
	publishStatus();
}

Status StatusPublisher::getStatus() const {
	return mStatus_;
}

void StatusPublisher::tick(const ros::WallTimerEvent& event) {
	publishStatus();
}

void StatusPublisher::publishStatus() const {
	known_map_localization::Status status;
	status.stamp = ros::Time::now();
	status.status = mStatus_;
	status.n_supporting_positions = mNSupportingGpsPos_;

	mStatusPublisher_.publish(status);
	pDataLogger_->logStatus(status);
}

void StatusPublisher::receiveSlamState(orb_slam::ORBState state) {
	if(state.state == 4) {
		// SLAM lost tracking
		mStatus_ == STATUS_NO_POS;
		ROS_WARN_THROTTLE(1., "ORB_SLAM lost tracking. No position available.");
		publishStatus();
	}
}

} /* namespace kml */
