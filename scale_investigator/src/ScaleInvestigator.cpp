/*
 * ScaleInvestigator.cpp
 *
 *  Created on: 01.08.2016
 *      Author: jacob
 */

#include "scale_investigator/ScaleInvestigator.h"
#include <opencv2/highgui/highgui.hpp>
#include <mapstitch/utils.h>

using namespace cv;

namespace known_map_localization {

ScaleInvestigator::ScaleInvestigator() : slamMapReceived(false), knownMapReceived(false),
		rotation(0), windowName("Scale Investigator") {
	namedWindow(windowName, WINDOW_AUTOSIZE);
	createTrackbar("Rotation", windowName, &rotation, 360,
			ScaleInvestigator::updateRotation, this);
	setMouseCallback(windowName, ScaleInvestigator::onMouse, this);
	ros::NodeHandle nh;
	slamMapSubscriber = nh.subscribe("slam_map", 1,
			&ScaleInvestigator::receiveSlamMap, this);
	knownMapSubscriber = nh.subscribe("known_map", 1,
			&ScaleInvestigator::receiveKnownMap, this);
}

void ScaleInvestigator::spin() {
	ROS_INFO("Scale Investigator started.");

	if(!(slamMapReceived && knownMapReceived)) {
		ROS_INFO("Wait for SLAM map and known map...");
	}

	while(!(slamMapReceived && knownMapReceived) && ros::ok()) {
		ros::spinOnce();
	}



	ROS_INFO("SLAM map and known map are available.");

	Point2f knownMapPoints[3];
	Point2f slamMapPoints[3];

	// draw both
	bothMaps = Mat(std::max(slamMap.rows, knownMap.rows), slamMap.cols + knownMap.cols, CV_8UC3);

	Rect knownMapRoi(0, 0, knownMap.cols, knownMap.rows);
	Rect slamMapRoi(knownMap.cols, 0, slamMap.cols, slamMap.rows);

	Mat left(bothMaps, knownMapRoi); // Copy constructor
	knownMap.copyTo(left);

	Mat right(bothMaps, slamMapRoi); // Copy constructor
	slamMap.copyTo(right);
	imshow(windowName, bothMaps);

	// first point pair
	MouseClick p00 = waitForMouseClick();
	assertInBounds(p00, knownMapRoi);
	circle(bothMaps, Point2i(p00.x, p00.y), 2, Scalar(255,0,0), -1);
	imshow(windowName, bothMaps);

	MouseClick p01 = waitForMouseClick();
	assertInBounds(p01, slamMapRoi);
	circle(bothMaps, Point2i(p01.x, p01.y), 2, Scalar(255,0,0), -1);
	line(bothMaps, Point2i(p00.x, p00.y), Point2i(p01.x, p01.y), Scalar(255,0,0), 1);
	imshow(windowName, bothMaps);
		// draw point and line

	// second point pair

	//third point pair

	// draw overlay

	// allow adjustment

	waitKey(0);
}

ScaleInvestigator::~ScaleInvestigator() {
	destroyWindow(windowName);
}

void ScaleInvestigator::updateRotation(int r, void *userData) {
	ScaleInvestigator *si = (ScaleInvestigator *)userData;
	assert(si);
	// todo rotate slam map
}

ScaleInvestigator::MouseClick ScaleInvestigator::waitForMouseClick() {
	mouseEventReceived = false;

	while(!mouseEventReceived && ros::ok()) {
		ros::spinOnce();
	}

	if(!ros::ok()) exit(0);

	return lastClick;
}

void ScaleInvestigator::onMouse(int event, int x, int y, int, void *userData) {
	ScaleInvestigator *si = (ScaleInvestigator *)userData;
	assert(si);

	if(event == EVENT_LBUTTONDOWN) {
		si->lastClick.x = x;
		si->lastClick.y = y;
		si->mouseEventReceived = true;
	}
}

void ScaleInvestigator::receiveSlamMap(nav_msgs::OccupancyGridPtr map) {
	assert(map);

	slamMap = occupancyGridToCvMat(map.get());
	slamMapResolution = map->info.resolution;

	slamMapSubscriber.shutdown();
	slamMapReceived = true;
}

void ScaleInvestigator::receiveKnownMap(nav_msgs::OccupancyGridPtr map) {
	assert(map);

	knownMap = occupancyGridToCvMat(map.get());
	knownMapResolution = map->info.resolution;

	knownMapSubscriber.shutdown();
	knownMapReceived = true;
}

void ScaleInvestigator::assertInBounds(MouseClick mc, Rect roi) {
	if(mc.x < roi.x || mc.x > roi.x + roi.width) {
		throw std::runtime_error("x out of bounds");
	}
	if(mc.y < roi.y || mc.y > roi.y + roi.height) {
		throw std::runtime_error("y out of bounds");
	}
}

} /* namespace known_map_localization */
