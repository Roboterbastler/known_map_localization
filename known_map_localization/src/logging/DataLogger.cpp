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
		mComputationId_(0),
		mSeparationChar_('\t'){
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

	mComputationsFile_ << ros::WallTime::now().toSec() << mSeparationChar_ << mComputationId_ << mSeparationChar_
			<< duration.toSec() << mSeparationChar_ << hypotheses.size() << mSeparationChar_
			<< slamMap.height * slamMap.width << mSeparationChar_
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

	mAlignmentsFile_ << h.stamp.toSec() << mSeparationChar_ << mComputationId_ << mSeparationChar_
			<< h.x << mSeparationChar_ << h.y << mSeparationChar_ << h.theta << mSeparationChar_ << h.scale << mSeparationChar_
			<< h.score << endl;
}

void DataLogger::logError(const known_map_localization::PoseError &error) {
	if (!mEnabled_)
		return;

	mErrorsFile_ << error.header.stamp.toSec() << mSeparationChar_
			<< error.translational_error << mSeparationChar_ << error.rotational_error
			<< endl;
}

void DataLogger::logScale(float scale, int mode) {
	if (!mEnabled_)
		return;

	mScalesFile_ << ros::WallTime::now().toSec() << mSeparationChar_ << scale << mSeparationChar_ << mode << endl;
}

void DataLogger::logFilter(const Alignment &filteredAlignment) {
	if (!mEnabled_)
		return;

	mFilterFile_ << ros::WallTime::now().toSec() << mSeparationChar_ << filteredAlignment.x << mSeparationChar_
			<< filteredAlignment.y << mSeparationChar_ << filteredAlignment.theta << mSeparationChar_
			<< filteredAlignment.scale << endl;
}

void DataLogger::logStatus(const known_map_localization::Status &status) {
	if (!mEnabled_)
		return;

	mStatusFile_ << ros::WallTime::now().toSec() << mSeparationChar_
			<< (int)status.status << mSeparationChar_
			<< (unsigned int)status.n_supporting_positions << endl;
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

	attributeNames << "Timestamp" << mSeparationChar_ << "Computation ID" << mSeparationChar_ << "X Translation" << mSeparationChar_
			<< "Y Translation" << mSeparationChar_ << "Rotation" << mSeparationChar_ << "Scale" << mSeparationChar_ << "Score";

	mAlignmentsFile_ << attributeNames.str() << endl;
}

void DataLogger::writeComputationsHeader() {
	stringstream attributeNames;

	attributeNames << "Time" << mSeparationChar_ << "ID" << mSeparationChar_ << "Duration" << mSeparationChar_
			<< "Number of hypotheses" << mSeparationChar_ << "SLAM map size" << mSeparationChar_
			<< "Known map size";

	mComputationsFile_ << attributeNames.str() << endl;
}

void DataLogger::writeErrorsHeader() {
	stringstream attributeNames;

	attributeNames << "Timestamp" << mSeparationChar_ << "Translational" << mSeparationChar_ << "Rotational";

	mErrorsFile_ << attributeNames.str() << endl;
}

void DataLogger::writeScalesHeader() {
	stringstream attributeNames;

	attributeNames << "Time" << mSeparationChar_ << "Scale" << mSeparationChar_ << "Mode";

	mScalesFile_ << attributeNames.str() << endl;
}

void DataLogger::writeFilterHeader() {
	stringstream attributeNames;

	attributeNames << "Time" << mSeparationChar_ << "X" << mSeparationChar_ << "Y" << mSeparationChar_ << "Rotation" << mSeparationChar_ << "Scale";

	mFilterFile_ << attributeNames.str() << endl;
}

void DataLogger::writeStatusHeader() {
	stringstream attributeNames;

	attributeNames << "Time" << mSeparationChar_ << "Status" << mSeparationChar_ << "Supporting GPS hints";

	mStatusFile_ << attributeNames.str() << endl;
}

std::string DataLogger::getLogFilePath(string name) {
	string path = ros::package::getPath("known_map_localization")
			+ "/data/analysis_data/";
	return path + name + ".tab";
}

} /* namespace kml */
