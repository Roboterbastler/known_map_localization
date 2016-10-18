/*
 * PassThroughFilter.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <filter/PassThroughFilter.h>

namespace kml {

PassThroughFilter::PassThroughFilter(SlamScaleManagerPtr pSlamScaleManager,
		StatusPublisherPtr pStatusPublisher,
		DataLoggerPtr pDataLogger) :
		Filter(pSlamScaleManager, pStatusPublisher, pDataLogger) {
	ROS_INFO("    Type: Pass through filter");
}

void PassThroughFilter::addHypotheses(const HypothesesVect &hypotheses) {
	if (hypotheses.empty()) {
		return;
	}

	mFilteredAlignment_ = hypotheses.front();
	pSlamScaleManager_->updateSlamScale(mFilteredAlignment_.scale);
	logAlignment(mFilteredAlignment_);
	mReady_ = true;
}

} /* namespace kml */
