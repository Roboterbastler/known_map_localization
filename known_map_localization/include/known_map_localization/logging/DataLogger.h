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
#include <SlamScaleManager.h>

namespace known_map_localization {
namespace logging {

class DataLogger;
typedef boost::shared_ptr<DataLogger> DataLoggerPtr;

/**
 * # Data Logger
 * Logs data to a CSV file to allow later analysis.
 *
 * ## Parameters
 * - **logging_enabled**: Enables/disables logging (default: enabled)
 */
class DataLogger {
public:
	static DataLoggerPtr instance();

	~DataLogger();

	void logComputation(const alignment::HypothesesVect &hypotheses,
			ros::WallDuration duration, const nav_msgs::MapMetaData &knownMap,
			const nav_msgs::MapMetaData &slamMap);

	void logHypothesis(const alignment::Hypothesis &h);

	void logError(const PoseError &error);

	void logScale(float scale, SlamScaleMode mode);

	void logFilter(const alignment::Alignment &filteredAlignment);

protected:
	DataLogger();

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

protected:
	/// Log file streams
	std::ofstream alignmentsFile;
	std::ofstream computationsFile;
	std::ofstream errorsFile;
	std::ofstream scalesFile;
	std::ofstream filterFile;

	/// ID of computations
	unsigned long computationId;

	/// flag indicating if logging is enabled/disabled
	bool enabled;

private:
	static DataLoggerPtr _instance;
};

} /* namespace logging */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOGGING_DATALOGGER_H_ */
