/*
 * Filter.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_

#include <alignment/Hypothesis.h>
#include <SlamScaleManager.h>
#include <logging/DataLogger.h>

namespace kml {

/**
 * # Filter
 * Base class for filter implementations, which choose from the given hypotheses
 * following specific strategies.
 */
class Filter {
public:
	Filter(SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger = DataLoggerPtr());

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

protected:
	/// the current filtered alignment
	Alignment mFilteredAlignment_;

	/// flag indicating if filtered alignment is available
	bool mReady_;

protected:
	SlamScaleManagerPtr pSlamScaleManager_;

private:
	DataLoggerPtr pDataLogger_;
};

typedef boost::shared_ptr<Filter> FilterPtr;
typedef boost::shared_ptr<Filter const> FilterConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_ */
