/*
 * DataLogger.cpp
 *
 *  Created on: 23.08.2016
 *      Author: jacob
 */

#include <string>
#include <algorithm>
#include <sstream>
#include <limits>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/package.h>

#include <logging/DataLogger.h>

namespace known_map_localization {
namespace logging {

using namespace std;

DataLoggerPtr DataLogger::_instance;

DataLogger::DataLogger() :
		computationId(0) {
	ROS_INFO("Data logger initialization...");

	ros::NodeHandle nh("~");

	enabled = nh.param("logging_enabled", true);

	if(!enabled) {
		ROS_INFO("    Data logging disabled.");
		return;
	}

	string alignmentsFileName = getLogFilePath("alignments");
	string computationsFileName = getLogFilePath("computations");
	string errorsFileName = getLogFilePath("errors");
	string scalesFileName = getLogFilePath("scales");
	string filterFileName = getLogFilePath("filter");

	alignmentsFile.open(alignmentsFileName.c_str());
	computationsFile.open(computationsFileName.c_str());
	errorsFile.open(errorsFileName.c_str());
	scalesFile.open(scalesFileName.c_str());
	filterFile.open(filterFileName.c_str());

	int precision = std::numeric_limits<double>::digits10 + 2;
	alignmentsFile.precision(precision);
	computationsFile.precision(precision);
	errorsFile.precision(precision);
	scalesFile.precision(precision);
	filterFile.precision(precision);

	ROS_INFO("    Opened files for logging.");

	writeHeader();

	ROS_INFO("    Wrote header rows.");
}

DataLogger::~DataLogger() {
	alignmentsFile.flush();
	computationsFile.flush();
	errorsFile.flush();
	scalesFile.flush();
	filterFile.flush();

	alignmentsFile.close();
	computationsFile.close();
	errorsFile.close();
	scalesFile.close();
	filterFile.close();
}

DataLoggerPtr DataLogger::instance() {
	if(!_instance) {
		_instance = DataLoggerPtr(new DataLogger());
	}
	return _instance;
}

void DataLogger::logComputation(const alignment::HypothesesVect &hypotheses,
		ros::WallDuration duration, const nav_msgs::MapMetaData &knownMap,
		const nav_msgs::MapMetaData &slamMap) {
	if(!enabled) return;

	ros::WallTime stamp = ros::WallTime::now();

	computationsFile << stamp.toSec() << '\t'
			<< computationId << '\t'
			<< duration.toSec() << '\t'
			<< hypotheses.size() << '\t'
			<< slamMap.height * slamMap.width << '\t'
			<< knownMap.height * knownMap.width << endl;

	// log alignments
	for (alignment::HypothesesVect::const_iterator it = hypotheses.begin();
			it != hypotheses.end(); ++it) {
		logHypothesis(*it);
	}

	computationId++;
}

void DataLogger::logHypothesis(const alignment::Hypothesis &h) {
	if(!enabled) return;

	alignmentsFile << h.stamp.toSec() << '\t'
			<< computationId << '\t'
			<< h.x << '\t'
			<< h.y << '\t'
			<< h.theta << '\t'
			<< h.scale << '\t'
			<< h.score << endl;
}

void DataLogger::logError(const PoseError &error) {
	if(!enabled) return;

	errorsFile << error.header.stamp.toSec() << '\t'
			<< error.translational_error << '\t'
			<< error.rotational_error << endl;
}

void DataLogger::logScale(float scale, SlamScaleMode mode) {
	if(!enabled) return;

	ROS_WARN("Logged scale...");

	scalesFile << ros::WallTime::now().toSec() << '\t'
			<< scale << '\t'
			<< mode << endl;
}

void DataLogger::logFilter(const alignment::Alignment &filteredAlignment) {
	if(!enabled) return;

	filterFile << ros::WallTime::now().toSec() << '\t'
			<< filteredAlignment.x << '\t'
			<< filteredAlignment.y << '\t'
			<< filteredAlignment.theta << '\t'
			<< filteredAlignment.scale << endl;
}

void DataLogger::writeHeader() {
	writeAlignmentsHeader();
	writeComputationsHeader();
	writeErrorsHeader();
	writeScalesHeader();
	writeFilterHeader();
}

void DataLogger::writeAlignmentsHeader() {
	stringstream attributeNames;//, types, optionalElements;

	attributeNames << "Timestamp\t"
			<< "Computation ID\t"
			<< "X Translation\t"
			<< "Y Translation\t"
			<< "Rotation\t"
			<< "Scale\t"
			<< "Score";

	alignmentsFile << attributeNames.str() << endl;
}

void DataLogger::writeComputationsHeader() {
	stringstream attributeNames;

	attributeNames << "Timestamp\t"
			<< "ID\t"
			<< "Duration\t"
			<< "Number of hypotheses\t"
			<< "SLAM map size\t"
			<< "Known map size";

	computationsFile << attributeNames.str() << endl;
}

void DataLogger::writeErrorsHeader() {
	stringstream attributeNames;

	attributeNames << "Timestamp\t"
			<< "Translational\t"
			<< "Rotational";

	errorsFile << attributeNames.str() << endl;
}

void DataLogger::writeScalesHeader() {
	stringstream attributeNames;

	attributeNames << "Timestamp\t"
			<< "Scale\t"
			<< "Mode";

	scalesFile << attributeNames.str() << endl;
}

void DataLogger::writeFilterHeader() {
	stringstream attributeNames;

	attributeNames << "Timestamp\t"
			<< "X\t"
			<< "Y\t"
			<< "Rotation\t"
			<< "Scale";

	filterFile << attributeNames.str() << endl;
}

std::string DataLogger::getLogFilePath(string name) {
	string path = ros::package::getPath("known_map_localization")
			+ "/data/analysis_data/";
	return path + name + ".tab";
}

} /* namespace logging */
} /* namespace known_map_localization */
