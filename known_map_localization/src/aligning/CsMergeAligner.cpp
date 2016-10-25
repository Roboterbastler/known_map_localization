/*
 * CsMergeAligner.cpp
 *
 *  Created on: 30.09.2016
 *      Author: jacob
 */

#include <aligning/CsMergeAligner.h>

namespace kml {

CsMergeAligner::CsMergeAligner(std::string serviceName) {
	ros::NodeHandle nh;
	mAligningClient_ = nh.serviceClient<cs_merge_msgs::getTransform>(serviceName);

	if(!mAligningClient_.exists()) {
		ROS_FATAL("CsMergeAligner service is not available: %s", mAligningClient_.getService().c_str());
	}
}

CsMergeAligner::~CsMergeAligner() {

}

HypothesesVect CsMergeAligner::align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap) {
	ros::WallTime start = ros::WallTime::now();
	ROS_INFO("Start aligning...");

	cs_merge_msgs::getTransformRequest request;
	request.map_one = *slamMap;
	request.map_two = *knownMap;

	cs_merge_msgs::getTransformResponse response;
	bool success = mAligningClient_.call(request, response);

	if(!success) {
		ROS_ERROR("CsMergeAligner service call failed.");
		return HypothesesVect();
	}

	// map origin to map origin computed by aligning
	tf::Transform slamOriginToKnownOrigin = transformCsToTf(response.result);
	slamOriginToKnownOrigin.getOrigin() *= knownMap->info.resolution;

	// SLAM map frame to SLAM map origin
	tf::Quaternion rotation;
	tf::Transform slamMapFrameToOrigin;
	slamMapFrameToOrigin.setOrigin(tf::Vector3(slamMap->info.origin.position.x, slamMap->info.origin.position.y, 0));
	tf::quaternionMsgToTF(slamMap->info.origin.orientation, rotation);
	slamMapFrameToOrigin.setRotation(rotation);

	// known map frame to known map origin
	tf::Transform knownMapFrameToOrigin;
	knownMapFrameToOrigin.setIdentity();
	knownMapFrameToOrigin.setOrigin(tf::Vector3(knownMap->info.origin.position.x, knownMap->info.origin.position.y, 0));
	tf::quaternionMsgToTF(knownMap->info.origin.orientation, rotation);
	knownMapFrameToOrigin.setRotation(rotation);

	// SLAM map frame to known map frame
	tf::Transform mapFrameToMapFrame = tf::Transform::getIdentity();
	mapFrameToMapFrame *= slamMapFrameToOrigin;
	mapFrameToMapFrame *= slamOriginToKnownOrigin;
	mapFrameToMapFrame *= knownMapFrameToOrigin.inverse();

	Hypothesis hypothesis;
	hypothesis.from = slamMap->header.frame_id;
	hypothesis.to = knownMap->header.frame_id;
	hypothesis.scale = 1.;
	hypothesis.score = 1.; // TODO: compute some kind of score (similar to mapmerge?)
	hypothesis.stamp = response.result.stamp;
	hypothesis.theta = tf::getYaw(mapFrameToMapFrame.getRotation());
	hypothesis.x = mapFrameToMapFrame.getOrigin().getX();
	hypothesis.y = mapFrameToMapFrame.getOrigin().getY();

	HypothesesVect hypothesisVector;
	hypothesisVector.push_back(hypothesis);

	ros::WallDuration duration = ros::WallTime::now() - start;
	ROS_INFO("Successfully completed aligning in %f sec", duration.toSec());
	return hypothesisVector;
}

tf::Transform transformCsToTf(const cs_merge_msgs::transform &transform) {
	tf::Transform t;
	t.setOrigin(tf::Vector3(transform.dx, transform.dy, 0));
	t.setRotation(tf::createQuaternionFromYaw(transform.rotation));
	return t;
}

} /* namespace kml */
