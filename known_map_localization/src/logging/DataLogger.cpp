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

namespace kml {

using namespace std;

DataLogger::DataLogger() :
		mComputationId_(0) {
	ROS_INFO("Data logger initialization...");

	ros::NodeHandle nh("~");

	mEnabled_ = nh.param("logging_enabled", true);

	if (!mEnabled_) {
		ROS_INFO("    Data logging disabled.");
		return;
	}

	string alignmentsFileName = getLogFilePath("alignments");
	string computationsFileName = getLogFilePath("computations");
	string errorsFileName = getLogFilePath("errors");
	string scalesFileName = getLogFilePath("scales");
	string filterFileName = getLogFilePath("filter");
	string statusFileName = getLogFilePath("status");

	mAlignmentsFile_.open(alignmentsFileName.c_str());
	mComputationsFile_.open(computationsFileName.c_str());
	mErrorsFile_.open(errorsFileName.c_str());
	mScalesFile_.open(scalesFileName.c_str());
	mFilterFile_.open(filterFileName.c_str());
	mStatusFile_.open(statusFileName.c_str());

	int precision = std::numeric_limits<double>::digits10 + 2;
	mAlignmentsFile_.precision(precision);
	mComputationsFile_.precision(precision);
	mErrorsFile_.precision(precision);
	mScalesFile_.precision(precision);
	mFilterFile_.precision(precision);
	mStatusFile_.precision(precision);

	ROS_INFO("    Opened files for logging.");

	writeHeader();

	ROS_INFO("    Wrote header rows.");
}

DataLogger::~DataLogger() {
	mAlignmentsFile_.flush();
	mComputationsFile_.flush();
	mErrorsFile_.flush();
	mScalesFile_.flush();
	mFilterFile_.flush();
	mStatusFile_.flush();

	mAlignmentsFile_.close();
	mComputationsFile_.close();
	mErrorsFile_.close();
	mScalesFile_.close();
	mFilterFile_.close();
	mStatusFile_.close();
}

void DataLogger::logComputation(const HypothesesVect &hypotheses,
		ros::WallDuration duration, const nav_msgs::MapMetaData &knownMap,
		const nav_msgs::MapMetaData &slamMap) {
	if (!mEnabled_)
		return;

	mComputationsFile_ << ros::WallTime::now().toSec() << '\t' << mComputationId_ << '\t'
			<< duration.toSec() << '\t' << hypotheses.size() << '\t'
			<< slamMap.height * slamMap.width << '\t'
			<< knownMap.height * knownMap.width << endl;

	// log alignments
	for (HypothesesVect::const_iterator it = hypotheses.begin();
			it != hypotheses.end(); ++it) {
		logHypothesis(*it);
	}

	mComputationId_++;
}

void DataLogger::logHypothesis(const Hypothesis &h) {
	if (!mEnabled_)
		return;

	mAlignmentsFile_ << h.stamp.toSec() << '\t' << mComputationId_ << '\t'
			<< h.x << '\t' << h.y << '\t' << h.theta << '\t' << h.scale << '\t'
			<< h.score << endl;
}

void DataLogger::logError(const known_map_localization::PoseError &error) {
	if (!mEnabled_)
		return;

	mErrorsFile_ << error.header.stamp.toSec() << '\t'
			<< error.translational_error << '\t' << error.rotational_error
			<< endl;
}

void DataLogger::logScale(float scale, int mode) {
	if (!mEnabled_)
		return;

	mScalesFile_ << ros::WallTime::now().toSec() << '\t' << scale << '\t' << mode << endl;
}

void DataLogger::logFilter(const Alignment &filteredAlignment) {
	if (!mEnabled_)
		return;

	mFilterFile_ << ros::WallTime::now().toSec() << '\t' << filteredAlignment.x << '\t'
			<< filteredAlignment.y << '\t' << filteredAlignment.theta << '\t'
			<< filteredAlignment.scale << endl;
}

void DataLogger::logStatus(const known_map_localization::Status &status) {
	if (!mEnabled_)
		return;

	mStatusFile_ << ros::WallTime::now().toSec() << '\t'
			<< status.status << '\t'
			<< status.n_supporting_positions << endl;
}

void DataLogger::writeHeader() {
	writeAlignmentsHeader();
	writeComputationsHeader();
	writeErrorsHeader();
	writeScalesHeader();
	writeFilterHeader();
	writeStatusHeader();
}

void DataLogger::writeAlignmentsHeader() {
	stringstream attributeNames; //, types, optionalElements;

	attributeNames << "Timestamp\t" << "Computation ID\t" << "X Translation\t"
			<< "Y Translation\t" << "Rotation\t" << "Scale\t" << "Score";

	mAlignmentsFile_ << attributeNames.str() << endl;
}

void DataLogger::writeComputationsHeader() {
	stringstream attributeNames;

	attributeNames << "Time\t" << "ID\t" << "Duration\t"
			<< "Number of hypotheses\t" << "SLAM map size\t"
			<< "Known map size";

	mComputationsFile_ << attributeNames.str() << endl;
}

void DataLogger::writeErrorsHeader() {
	stringstream attributeNames;

	attributeNames << "Timestamp\t" << "Translational\t" << "Rotational";

	mErrorsFile_ << attributeNames.str() << endl;
}

void DataLogger::writeScalesHeader() {
	stringstream attributeNames;

	attributeNames << "Time\t" << "Scale\t" << "Mode";

	mScalesFile_ << attributeNames.str() << endl;
}

void DataLogger::writeFilterHeader() {
	stringstream attributeNames;

	attributeNames << "Time\t" << "X\t" << "Y\t" << "Rotation\t" << "Scale";

	mFilterFile_ << attributeNames.str() << endl;
}

void DataLogger::writeStatusHeader() {
	stringstream attributeNames;

	attributeNames << "Time\t" << "Status\t" << "Supporting GPS hints";

	mStatusFile_ << attributeNames.str() << endl;
}

std::string DataLogger::getLogFilePath(string name) {
	string path = ros::package::getPath("known_map_localization")
			+ "/data/analysis_data/";
	return path + name + ".tab";
}

} /* namespace kml */
