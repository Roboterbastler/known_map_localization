/*
 * MapstitchAligner.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <ros/ros.h>
#include <mapstitch/mapstitch.h>
#include <mapstitch/utils.h>

#include <aligning/MapstitchAligner.h>
#include <Exception.h>
#include <Utils.h>

namespace kml {

HypothesesVect MapstitchAligner::align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap) {
	ros::NodeHandle nh("~");

	float max_pairwise_distance;
	if(!nh.getParam("max_pairwise_distance", max_pairwise_distance)) {
		throw AlignerMissingParameter("Missing mapstitch parameter: max_pairwise_distance");
	}

	ros::WallTime startTime = ros::WallTime::now();

	ROS_INFO("Start aligning...");

	cv::Mat knownMapImg = occupancyGridToCvMat(knownMap.get());
	cv::Mat slamMapImg = occupancyGridToCvMat(slamMap.get());

	std::auto_ptr<StitchedMap> stitchedMap;

	try {
		// get transformation between images
		stitchedMap = std::auto_ptr<StitchedMap>(new StitchedMap(slamMapImg, knownMapImg, max_pairwise_distance));
	} catch(std::exception &e) {
		throw AlignerInternalError("Map stitching failed because exception occurred: " + std::string(e.what()));
	}

	if (!stitchedMap->is_valid) {
		throw AlignerFailed("No valid alignment found");
	}

	StampedAlignment alignment;

	// let scale be the mean of x and y scale, assuming both are approximately equal
	alignment.scale = (stitchedMap->scalex + stitchedMap->scaley) / 2.0;
	if (std::fabs(stitchedMap->scalex - stitchedMap->scaley) > 0.1) {
		ROS_ERROR(
				"Mapstitch returned significantly different scales for x and y. Only uniform scaling is supported: scale_x=%f but scale_y=%f",
				stitchedMap->scalex, stitchedMap->scaley);
		throw AlignerInternalError("Different scales");
	}

	ROS_DEBUG("Raw transformation: x=%f y=%f yaw=%f degree scale=%f", stitchedMap->transx, stitchedMap->transy, stitchedMap->rot_deg, alignment.scale);

	// combine transformations to get transformation from the SLAM map's origin to the known map's origin
	// note: inverted direction of rotation! (different coordinate systems)
	tf::Transform slamMap_to_knownMap(
			tf::createQuaternionFromYaw(-stitchedMap->rot_rad),
			tf::Vector3(stitchedMap->transx, stitchedMap->transy, 0));

	// translation is given in pixels of the slam map
	slamMap_to_knownMap.getOrigin() *= knownMap->info.resolution / alignment.scale;

	// origin offsets
	tf::Transform knownMapOrigin_to_knownMapAnchorFrame = getOriginTransform(knownMap).inverse();
	tf::Transform slamMapFrame_to_slamMapOrigin = getOriginTransform(slamMap);

	// scale factor for SLAM coordinates
	const float oldSlamResolution = slamMap->info.resolution;
	const float newSlamResolution = knownMap->info.resolution / alignment.scale;
	alignment.scale = newSlamResolution / oldSlamResolution;
	slamMapFrame_to_slamMapOrigin.getOrigin() *= alignment.scale;

	ROS_DEBUG("Transform from %s to SLAM map origin: x=%f y=%f yaw=%f",
			slamMap->header.frame_id.c_str(),
			slamMapFrame_to_slamMapOrigin.getOrigin().getX(),
			slamMapFrame_to_slamMapOrigin.getOrigin().getY(),
			radToDeg(tf::getYaw(slamMapFrame_to_slamMapOrigin.getRotation())));
	ROS_DEBUG("Transform from SLAM map to known map: x=%f y=%f yaw=%f scale=%f",
			slamMap_to_knownMap.getOrigin().getX(),
			slamMap_to_knownMap.getOrigin().getY(),
			radToDeg(tf::getYaw(slamMap_to_knownMap.getRotation())),
			alignment.scale);
	ROS_DEBUG("Transform from known map origin to %s: x=%f y=%f yaw=%f",
			knownMap->header.frame_id.c_str(),
			knownMapOrigin_to_knownMapAnchorFrame.getOrigin().getX(),
			knownMapOrigin_to_knownMapAnchorFrame.getOrigin().getY(),
			radToDeg(tf::getYaw(knownMapOrigin_to_knownMapAnchorFrame.getRotation())));

	// put everything together
	tf::Transform slamMapFrame_to_knownMapAnchorFrame;
	slamMapFrame_to_knownMapAnchorFrame.setIdentity();
	slamMapFrame_to_knownMapAnchorFrame *= slamMapFrame_to_slamMapOrigin;
	slamMapFrame_to_knownMapAnchorFrame *= slamMap_to_knownMap;
	slamMapFrame_to_knownMapAnchorFrame *= knownMapOrigin_to_knownMapAnchorFrame;

	// pack alignment
	alignment.from = slamMap->header.frame_id;
	alignment.to = knownMap->header.frame_id;
	alignment.stamp = ros::Time::now();
	alignment.theta = tf::getYaw(slamMapFrame_to_knownMapAnchorFrame.getRotation());
	alignment.x = slamMapFrame_to_knownMapAnchorFrame.getOrigin().getX();
	alignment.y = slamMapFrame_to_knownMapAnchorFrame.getOrigin().getY();

	ROS_INFO("Transform from %s to %s: x=%f y=%f yaw=%f", alignment.from.c_str(), alignment.to.c_str(),
			alignment.x, alignment.y, radToDeg(alignment.theta));

	ros::WallDuration duration = ros::WallTime::now() - startTime;

	ROS_INFO("Successfully completed aligning in %f sec", duration.toSec());

	// No score available, so all are scored 1
	HypothesesVect hypotheses(1, Hypothesis(alignment, 1.0));
	return hypotheses;
}

tf::Transform getOriginTransform(nav_msgs::OccupancyGridConstPtr map) {
	tf::Quaternion orientation;
	tf::quaternionMsgToTF(map->info.origin.orientation, orientation);
	return tf::Transform(orientation,
			tf::Vector3(map->info.origin.position.x, map->info.origin.position.y,
					map->info.origin.position.z));
}

} /* namespace kml */
