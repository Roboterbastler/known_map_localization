/*
 * Filter.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_

#include <ros/wall_timer.h>
#include <tf/transform_broadcaster.h>

#include <alignment/Hypothesis.h>
#include <SlamScaleManager.h>
#include <logging/DataLogger.h>
#include <StatusPublisher.h>

namespace kml {

/**
 * # Filter
 * Base class for filter implementations, which choose from the given hypotheses
 * following specific strategies.
 *
 * ## Parameters
 * - **map_transform_rate**: Rate for publishing the map transform (updates per second). Defaults to 10.
 *
 * ## Published Topics
 * - **tf**: Map transformation between map frames
 */
class Filter {
public:
	Filter(SlamScaleManagerPtr pSlamScaleManager, StatusPublisherPtr pStatusPublisher, DataLoggerPtr pDataLogger = DataLoggerPtr());

	virtual ~Filter();

	/**
	 * Add new hypotheses and update the filtered alignment accordingly.
	 * Multiple possible implementations are given by subclasses of Filter.
	 * @param hypotheses New hypotheses
	 */
	virtual void addHypotheses(const HypothesesVect &hypotheses) = 0;

	/**
	 * Get the filtered alignment, if available. If it isn't available, an exception is thrown.
	 * @return The filtered alignment
	 * @throw AlignmentNotAvailable, if filtered alignment is not available
	 */
	virtual const Alignment& getAlignment() const;

	/**
	 * Check if the filtered alignment is available.
	 * @return State of the filtered alignment
	 */
	bool isAvailable() const;

protected:
	/**
	 * Logs the alignment using the data logger.
	 * @param alignment The alignment to log
	 */
	void logAlignment(const Alignment &alignment);

	/**
	 * Updates the tf transform from the SLAM map frame to the known map frame using current alignment.
	 */
	void updateMapTransform();

private:
	void tick(const ros::WallTimerEvent& event);

protected:
	/// the current filtered alignment
	Alignment mFilteredAlignment_;

	/// flag indicating if filtered alignment is available
	bool mReady_;

private:
	/// Used to regularly update map transformation
	ros::WallTimer mTimer_;

	/// Broadcaster for tf transforms
	tf::TransformBroadcaster mBroadcaster_;

protected:
	SlamScaleManagerPtr pSlamScaleManager_;
	StatusPublisherPtr pStatusPublisher_;

private:
	DataLoggerPtr pDataLogger_;
};

typedef boost::shared_ptr<Filter> FilterPtr;
typedef boost::shared_ptr<Filter const> FilterConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_ */
