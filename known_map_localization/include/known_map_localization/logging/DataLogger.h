/*
 * DataLogger.h
 *
 *  Created on: 23.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOGGING_DATALOGGER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOGGING_DATALOGGER_H_

#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>

#include <alignment/Hypothesis.h>

namespace known_map_localization {
namespace logging {

/**
 * Logs data to a CSV file to allow later analysis.
 */
class DataLogger {
public:
	DataLogger(bool enabled = true);
	~DataLogger();

	void logComputation(const alignment::HypothesesVect &hypotheses,
			ros::WallDuration duration, nav_msgs::MapMetaData knownMap,
			nav_msgs::MapMetaData slamMap);

private:
	/**
	 * Determines the file name where to log data. Part of the name is the current time.
	 * @param name Name to be integrated in the final file name
	 * @return File name
	 */
	static std::string getLogFileName(std::string name);

	/**
	 * Write the Orange native format
	 */
	void writeOrangeHeader();

	/// Log file streams
	std::ofstream alignmentsFile;
	std::ofstream computationsFile;

	unsigned int computationId;

	bool enabled;
};

} /* namespace logging */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_LOGGING_DATALOGGER_H_ */
