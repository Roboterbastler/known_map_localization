/*
 * KnownMapLocalization.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <KnownMapLocalization.h>
#include <filter/PassThroughFilter.h>
#include <alignment/Hypothesis.h>
#include <Utils.h>
#include <Exception.h>

namespace known_map_localization {
using namespace known_map_server;
using namespace alignment;

KnownMapLocalization::KnownMapLocalization() :
		algorithmSelector(new AlgorithmSelector()),
		knownMapServer(new KnownMapServer(algorithmSelector->getKnownMapPreprocessor())),
		filter(new filter::PassThroughFilter()),
		baseLinkPublisher(new base_link::BaseLinkPublisher(filter, knownMapServer->getAnchor(), ros::WallDuration(0.2))),
		visualizationSlamMapPublisher(new visualization::VisualizationSlamMapPublisher(filter)),
		dataLogger(true) {
	aligner = algorithmSelector->getAligner();
	knownMapPreprocessor = algorithmSelector->getKnownMapPreprocessor();
	slamMapPreprocessor = algorithmSelector->getSlamMapPreprocessor();

	// subscribe to topics
	ros::NodeHandle nh("~");
	slamMapSubscriber = nh.subscribe("slam_map", 1, &KnownMapLocalization::receiveSlamMap, this);
}

void KnownMapLocalization::receiveSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap) {
	// TODO: initialize orientation for slam map (workaround until fixed in ORB_SLAM)
	nav_msgs::OccupancyGridPtr slamMapFixed(new nav_msgs::OccupancyGrid(*slamMap));
	tf::quaternionTFToMsg(tf::Quaternion(0, 0, 0, 1), slamMapFixed->info.origin.orientation);

	// preprocess the SLAM map
	if(!slamMapPreprocessor->process(slamMapFixed)) {
		// preprocessing failed
		ROS_WARN("SLAM map preprocessing failed. Aligning cancelled.");
		return;
	}

	// publish visualization SLAM map
	visualizationSlamMapPublisher->publishSlamMap(slamMapFixed);

	try {
		ros::WallTime start = ros::WallTime::now();

		// compute hypotheses (alignments)
		HypothesesVect hypotheses = aligner->align(knownMapServer->getKnownMap(), slamMapFixed);

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

		dataLogger.logComputation(hypotheses, duration, knownMapServer->getKnownMap()->info, slamMapFixed->info);

		filter->addHypotheses(hypotheses);
	} catch(AlignerInternalError &e) {
		ROS_WARN_STREAM("Internal aligner error: " << e.what());
	} catch(AlignerFailed &e) {
		ROS_DEBUG("Aligning failed.");
		return;
	}
}

} /* namespace known_map_localization */
