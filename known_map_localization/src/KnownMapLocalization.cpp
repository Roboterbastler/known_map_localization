/*
 * KnownMapLocalization.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <KnownMapLocalization.h>

#include <Exception.h>
#include <Utils.h>
#include <factory/KmlMapmergeFactory.h>
#include <factory/KmlCsMergeIcpGradientFactory.h>
#include <factory/KmlCsMergeIcpSvdFactory.h>
#include <factory/KmlCsMergeHoughCornerFactory.h>
#include <factory/KmlCsMergeHoughCcrFactory.h>
#include <factory/KmlMapstitchFactory.h>

namespace kml {

KnownMapLocalization::KnownMapLocalization() : mRate_(2.0), mLastProcessing_(0) {
	// create factory accordingly
	KmlFactoryConstPtr factory = selectStrategy();

	// let the factory do it's work...
	pDataLogger_ = factory->createDataLogger();
	pStatusPublisher_ = factory->createStatusPublisher(pDataLogger_);
	pKnownMapPreprocessor_ = factory->createKnownMapPreprocessor();
	pAligner_ = factory->createAligner();
	pKnownMapServer_ = factory->createKnownMapServer(pKnownMapPreprocessor_);
	pGpsManager_ = factory->createGpsManager(pKnownMapServer_);
	pSlamScaleManager_ = factory->createSlamScaleManager(pGpsManager_, pDataLogger_);
	pSlamMapPreprocessor_ = factory->createSlamMapPreprocessor(pSlamScaleManager_, pKnownMapServer_);
	pFilter_ = factory->createFilter(pGpsManager_, pKnownMapServer_, pSlamScaleManager_, pStatusPublisher_, pDataLogger_);
	pLocalization_ = factory->createLocalization(pFilter_, pSlamScaleManager_, pKnownMapServer_);
	pPoseErrorPublisher_ = factory->createPoseErrorPublisher(pDataLogger_);

	// subscribe to topics
	ros::NodeHandle nh("~");
	mSlamMapSubscriber_ = nh.subscribe("slam_map", 1, &KnownMapLocalization::receiveSlamMap, this);
	mSlamMapPublisher_ = nh.advertise<nav_msgs::OccupancyGrid>("visualization_slam_map", 10);

	// initialization done
	pStatusPublisher_->setStatus(STATUS_NO_POS);
}

KmlFactoryConstPtr KnownMapLocalization::selectStrategy() const {
	ros::NodeHandle nh("~");
	std::string algorithmSpecifier;

	if(!nh.getParam("algorithm", algorithmSpecifier)) {
		throw AlgorithmNotSpecified("Algorithm parameter is missing");
	}

	ROS_INFO("Algorithm: %s", algorithmSpecifier.c_str());

	if(algorithmSpecifier == "mapmerge") {
		return boost::make_shared<KmlMapmergeFactory>();
	} if(algorithmSpecifier == "mapstitch") {
		return boost::make_shared<KmlMapstitchFactory>();
	} if(algorithmSpecifier == "cs_merge_icp_gradient") {
		return boost::make_shared<KmlCsMergeIcpGradientFactory>();
	} if(algorithmSpecifier == "cs_merge_icp_svd") {
		return boost::make_shared<KmlCsMergeIcpSvdFactory>();
	} if(algorithmSpecifier == "cs_merge_hough_corner") {
		return boost::make_shared<KmlCsMergeHoughCornerFactory>();
	} if(algorithmSpecifier == "cs_merge_hough_ccr") {
		return boost::make_shared<KmlCsMergeHoughCcrFactory>();
	} else {
		ROS_FATAL("Illegal algorithm specifier: %s", algorithmSpecifier.c_str());
		ROS_BREAK();
		return KmlFactoryConstPtr();
	}
}

void KnownMapLocalization::receiveSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap) {
	if((ros::WallTime::now() - mLastProcessing_).toSec() < mRate_.toSec()) {
		ROS_DEBUG("Skipped SLAM map...");
		return;
	}
	mLastProcessing_ = ros::WallTime::now();

	// TODO: initialize orientation for slam map (workaround until fixed in ORB_SLAM)
	nav_msgs::OccupancyGridPtr slamMapFixed(new nav_msgs::OccupancyGrid(*slamMap));
	tf::quaternionTFToMsg(tf::Quaternion(0, 0, 0, 1), slamMapFixed->info.origin.orientation);

	// preprocess the SLAM map
	if(!pSlamMapPreprocessor_->process(slamMapFixed)) {
		// preprocessing failed
		ROS_WARN("SLAM map preprocessing failed. Aligning cancelled.");
		return;
	}

	// publish visualization SLAM map
	mSlamMapPublisher_.publish(slamMapFixed);

	try {
		ros::WallTime start = ros::WallTime::now();

		// compute hypotheses (alignments)
		HypothesesVect hypotheses = pAligner_->align(pKnownMapServer_->getKnownMap(), slamMapFixed);

		ros::WallDuration duration = ros::WallTime::now() - start;

		ROS_INFO("Got %ld new hypotheses.", hypotheses.size());

		for(HypothesesVect::const_iterator it = hypotheses.begin(); it != hypotheses.end(); ++it) {
			ROS_DEBUG("  - [x=%.2f, y=%.2f, theta=%.2f, scale=%.2f]. Score = %.4f",
					it->x,
					it->y,
					radToDeg(it->theta),
					it->scale,
					it->score);
		}

		pDataLogger_->logComputation(hypotheses, duration, pKnownMapServer_->getKnownMap()->info, slamMapFixed->info);

		pFilter_->addHypotheses(hypotheses);
		pLocalization_->localize();
	} catch(AlignerInternalError &e) {
		ROS_WARN_STREAM("Internal aligner error: " << e.what());
	} catch(AlignerFailed &e) {
		ROS_WARN("Aligning failed: %s", e.what());
		return;
	}
}

} /* namespace kml */
