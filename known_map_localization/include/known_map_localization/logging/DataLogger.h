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
#include <known_map_localization/Status.h>

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

	void logStatus(const known_map_localization::Status &status);

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

	/**
	 * Write the table header for the status table
	 */
	void writeStatusHeader();

private:
	/// Log file streams
	std::ofstream mAlignmentsFile_;
	std::ofstream mComputationsFile_;
	std::ofstream mErrorsFile_;
	std::ofstream mScalesFile_;
	std::ofstream mFilterFile_;
	std::ofstream mStatusFile_;

	/// ID of computations
	unsigned long mComputationId_;

	/// flag indicating if logging is enabled/disabled
	bool mEnabled_;

	/// Char used to separate values in the log file
	char mSeparationChar_;
};

typedef boost::shared_ptr<DataLogger> DataLoggerPtr;
typedef boost::shared_ptr<DataLogger const> DataLoggerConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOGGING_DATALOGGER_H_ */
