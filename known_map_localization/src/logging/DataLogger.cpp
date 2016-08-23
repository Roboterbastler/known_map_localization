/*
 * DataLogger.cpp
 *
 *  Created on: 23.08.2016
 *      Author: jacob
 */

#include <string>
#include <algorithm>
#include <sstream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/package.h>

#include <logging/DataLogger.h>

namespace known_map_localization {
namespace logging {

using namespace std;

DataLogger::DataLogger(bool enabled) :
		computationId(0), enabled(enabled) {
	if(!enabled) {
		ROS_INFO("Data logging disabled.");
		return;
	}

	string alignmentsFileName = getLogFileName("alignments");
	string computationsFileName = getLogFileName("computations");

	alignmentsFile.open(alignmentsFileName.c_str());
	computationsFile.open(computationsFileName.c_str());

	ROS_INFO("Logging alignment data to %s.", alignmentsFileName.c_str());
	ROS_INFO("Logging alignment data to %s.", computationsFileName.c_str());

	if (!alignmentsFile.is_open()) {
		ROS_WARN("Logging data failed. Unable to open file %s",
				alignmentsFileName.c_str());
	}
	if (!computationsFile.is_open()) {
		ROS_WARN("Logging data failed. Unable to open file %s",
				computationsFileName.c_str());
	}

	writeOrangeHeader();
}

DataLogger::~DataLogger() {
	alignmentsFile.close();
	computationsFile.close();
}

void DataLogger::logComputation(const alignment::HypothesesVect &hypotheses,
		ros::WallDuration duration, nav_msgs::MapMetaData knownMap,
		nav_msgs::MapMetaData slamMap) {
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
		alignmentsFile << stamp.toSec() << '\t'
				<< computationId << '\t'
				<< it->x << '\t'
				<< it->y << '\t'
				<< it->theta << '\t'
				<< it->scale << '\t'
				<< it->score << endl;
	}

	computationId++;
}

void DataLogger::writeOrangeHeader() {
	stringstream attributeNames, types, optionalElements;

	// timestamp
	attributeNames << "Timestamp\t";
	types << "continuous\t";
	optionalElements << "\t";

	// computation ID
	attributeNames << "Computation ID\t";
	types << "discrete\t";
	optionalElements << "\t";

	// X
	attributeNames << "X Translation\t";
	types << "continuous\t";
	optionalElements << "\t";

	// Y
	attributeNames << "Y Translation\t";
	types << "continuous\t";
	optionalElements << "\t";

	// Rotation
	attributeNames << "Rotation\t";
	types << "continuous\t";
	optionalElements << "\t";

	// Scale
	attributeNames << "Scale\t";
	types << "continuous\t";
	optionalElements << "\t";

	// Score
	attributeNames << "Score";
	types << "continuous";

	alignmentsFile << attributeNames.str() << endl << types.str() << endl
			<< optionalElements.str() << endl;
	attributeNames = stringstream();
	types = stringstream();
	optionalElements = stringstream();

	// computations file
	// timestamp
	attributeNames << "Timestamp\t";
	types << "continuous\t";
	optionalElements << "\t";

	// computation ID
	attributeNames << "ID\t";
	types << "discrete\t";
	optionalElements << "\t";

	// duration
	attributeNames << "Duration\t";
	types << "continuous\t";
	optionalElements << "\t";

	// number of hypotheses
	attributeNames << "Number of hypotheses\t";
	types << "discrete\t";
	optionalElements << "\t";

	// SLAM map size
	attributeNames << "SLAM map size\t";
	types << "discrete\t";
	optionalElements << "\t";

	// known map size
	attributeNames << "Known map size";
	types << "discrete";

	computationsFile << attributeNames.str() << endl << types.str() << endl
			<< optionalElements.str() << endl;
	attributeNames.clear();
	types.clear();
	optionalElements.clear();
}

std::string DataLogger::getLogFileName(string name) {
	ros::WallTime currentTime(floor(ros::WallTime::now().toSec()));
	string dateTimeStr = boost::posix_time::to_simple_string(
			currentTime.toBoost());
	replace(dateTimeStr.begin(), dateTimeStr.end(), ' ', '_');

	string path = ros::package::getPath("known_map_localization")
			+ "/data/analysis_data/";

	return path + dateTimeStr + '_' + name + ".tab";
}

} /* namespace logging */
} /* namespace known_map_localization */
