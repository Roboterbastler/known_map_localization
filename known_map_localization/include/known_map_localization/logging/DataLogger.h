/*
 * DataLogger.h
 *
 *  Created on: 23.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOGGING_DATALOGGER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOGGING_DATALOGGER_H_

#include <fstream>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>

#include <alignment/Hypothesis.h>
#include <known_map_localization/PoseError.h>

namespace kml {

/**
 * # Data Logger
 * Logs data to a CSV file to allow later analysis.
 *
 * ## Parameters
 * - **logging_enabled**: Enables/disables logging (default: enabled)
 */
class DataLogger {
public:
	DataLogger();

	~DataLogger();

	void logComputation(const HypothesesVect &hypotheses,
			ros::WallDuration duration, const nav_msgs::MapMetaData &knownMap,
			const nav_msgs::MapMetaData &slamMap);

	void logHypothesis(const Hypothesis &h);

	void logError(const known_map_localization::PoseError &error);

	void logScale(float scale, int mode);

	void logFilter(const Alignment &filteredAlignment);

protected:

	/**
	 * Determines the full file path where to log data.
	 * @param name File name
	 * @return File path
	 */
	static std::string getLogFilePath(std::string name);

	/**
	 * Write the table headers
	 */
	void writeHeader();

	/**
	 * Write the table header for the alignments table
	 */
	void writeAlignmentsHeader();

	/**
	 * Write the table header for the computations table
	 */
	void writeComputationsHeader();

	/**
	 * Write the table header for the errors table
	 */
	void writeErrorsHeader();

	/**
	 * Write the table header for the scales table
	 */
	void writeScalesHeader();

	/**
	 * Write the table header for the filter table
	 */
	void writeFilterHeader();

private:
	/// Log file streams
	std::ofstream mAlignmentsFile_;
	std::ofstream mComputationsFile_;
	std::ofstream mErrorsFile_;
	std::ofstream mScalesFile_;
	std::ofstream mFilterFile_;

	/// ID of computations
	unsigned long mComputationId_;

	/// flag indicating if logging is enabled/disabled
	bool mEnabled_;

	/// Initialization time used as a reference for logged events
	ros::WallTime mReferenceTime_;
};

typedef boost::shared_ptr<DataLogger> DataLoggerPtr;
typedef boost::shared_ptr<DataLogger const> DataLoggerConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOGGING_DATALOGGER_H_ */
