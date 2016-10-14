/*
 * Filter.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <filter/Filter.h>

#include <Exception.h>

namespace kml {

Filter::Filter(SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger) :
		mReady_(false), pSlamScaleManager_(pSlamScaleManager), pDataLogger_(
				pDataLogger) {
	ROS_ASSERT(pSlamScaleManager_);

	ROS_INFO("Filter initialization...");
}

Filter::~Filter() {
}

const Alignment& Filter::getAlignment() const {
	if (!mReady_) {
		throw AlignmentNotAvailable("Filtered alignment is not yet available");
	}
	return mFilteredAlignment_;
}

bool Filter::isAvailable() const {
	return mReady_;
}

void Filter::logAlignment(const Alignment &alignment) {
	if (pDataLogger_) {
		pDataLogger_->logFilter(alignment);
	}
}

} /* namespace kml */
