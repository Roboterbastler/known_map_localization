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
		baseLinkPublisher(new base_link::BaseLinkPublisher(filter, ros::WallDuration(0.2))),
		visualizationSlamMapPublisher(new visualization::VisualizationSlamMapPublisher(filter)) {
	aligner = algorithmSelector->getAligner();
	knownMapPreprocessor = algorithmSelector->getKnownMapPreprocessor();
	slamMapPreprocessor = algorithmSelector->getSlamMapPreprocessor();

	// subscribe to topics
	ros::NodeHandle nh("~");
	slamMapSubscriber = nh.subscribe("slam_map", 10, &KnownMapLocalization::receiveSlamMap, this);
}

void KnownMapLocalization::receiveSlamMap(const nav_msgs::OccupancyGridConstPtr &slamMap) {
	// TODO: initialize orientation for slam map (workaround until fixed in ORB_SLAM)
	nav_msgs::OccupancyGridPtr slamMapFixed(new nav_msgs::OccupancyGrid(*slamMap));
	tf::quaternionTFToMsg(tf::Quaternion(0, 0, 0, 1), slamMapFixed->info.origin.orientation);

	// preprocess the SLAM map
	slamMapPreprocessor->process(slamMapFixed);

	try {
		// compute hypotheses (alignments)
		HypothesesVect hypotheses = aligner->align(knownMapServer->getKnownMap(), slamMapFixed);

		ROS_INFO("Got %ld new hypotheses.", hypotheses.size());

		for(HypothesesVect::const_iterator it = hypotheses.begin(); it != hypotheses.end(); ++it) {
			ROS_INFO("  - Transform from %s to %s [x=%f, y=%f, theta=%f, scale=%f]. Score = %f",
					it->from.c_str(),
					it->to.c_str(),
					it->x,
					it->y,
					radToDeg(it->theta),
					it->scale,
					it->score);
		}

		filter->addHypotheses(hypotheses);
	} catch(AlignerInternalError &e) {
		ROS_WARN_STREAM("Internal aligner error: " << e.what());
	} catch(AlignerFailed &e) {
		ROS_DEBUG("Aligning failed.");
		return;
	}
}

} /* namespace known_map_localization */
