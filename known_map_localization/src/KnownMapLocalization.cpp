/*
 * KnownMapLocalization.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <KnownMapLocalization.h>
#include <known_map_server/KnownMapServer.h>
#include <preprocessing/SlamMapPreprocessor.h>
#include <visualization/VisualizationSlamMapPublisher.h>
#include <preprocessing/KnownMapPreprocessor.h>
#include <base_link/BaseLinkPublisher.h>
#include <SlamScaleManager.h>
#include <aligning/Aligner.h>
#include <alignment/Hypothesis.h>
#include <Utils.h>
#include <Exception.h>

namespace known_map_localization {
using namespace known_map_server;
using namespace alignment;
using namespace preprocessing;

KnownMapLocalization::KnownMapLocalization() :
		dataLogger(true) {

	// initialize Singletons
	KnownMapServer::instance();
	aligning::Aligner::instance();
	KnownMapPreprocessor::instance();
	SlamMapPreprocessor::instance();
	SlamScaleManager::instance();
	base_link::BaseLinkPublisher::instance();

	// subscribe to topics
	ros::NodeHandle nh("~");
	slamMapSubscriber = nh.subscribe("slam_map", 1, &KnownMapLocalization::receiveSlamMap, this);
}

void KnownMapLocalization::receiveSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap) {
	// TODO: initialize orientation for slam map (workaround until fixed in ORB_SLAM)
	nav_msgs::OccupancyGridPtr slamMapFixed(new nav_msgs::OccupancyGrid(*slamMap));
	tf::quaternionTFToMsg(tf::Quaternion(0, 0, 0, 1), slamMapFixed->info.origin.orientation);

	// preprocess the SLAM map
	if(!SlamMapPreprocessor::instance()->process(slamMapFixed)) {
		// preprocessing failed
		ROS_WARN("SLAM map preprocessing failed. Aligning cancelled.");
		return;
	}

	// publish visualization SLAM map
	visualization::VisualizationSlamMapPublisher::instance()->publishSlamMap(slamMapFixed);

	try {
		ros::WallTime start = ros::WallTime::now();

		// compute hypotheses (alignments)
		HypothesesVect hypotheses = aligning::Aligner::instance()->align(KnownMapServer::instance()->getKnownMap(), slamMapFixed);

		ros::WallDuration duration = ros::WallTime::now() - start;

		ROS_INFO("Got %ld new hypotheses.", hypotheses.size());

		for(HypothesesVect::const_iterator it = hypotheses.begin(); it != hypotheses.end(); ++it) {
			ROS_INFO("  - [x=%.2f, y=%.2f, theta=%.2f, scale=%.2f]. Score = %.4f",
					it->x,
					it->y,
					radToDeg(it->theta),
					it->scale,
					it->score);
		}

		dataLogger.logComputation(hypotheses, duration, KnownMapServer::instance()->getKnownMap()->info, slamMapFixed->info);

		filter::Filter::instance()->addHypotheses(hypotheses);
	} catch(AlignerInternalError &e) {
		ROS_WARN_STREAM("Internal aligner error: " << e.what());
	} catch(AlignerFailed &e) {
		ROS_DEBUG("Aligning failed.");
		return;
	}
}

} /* namespace known_map_localization */
