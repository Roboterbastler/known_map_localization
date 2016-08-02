/*
 * ScaleInvestigator.h
 *
 *  Created on: 01.08.2016
 *      Author: jacob
 */

#ifndef SCALE_INVESTIGATOR_SRC_SCALEINVESTIGATOR_H_
#define SCALE_INVESTIGATOR_SRC_SCALEINVESTIGATOR_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace known_map_localization {

class ScaleInvestigator {
public:
	ScaleInvestigator();
	~ScaleInvestigator();

	void spin();

	class MouseClick {
	public:
		int x;
		int y;
	};

private:
	static void updateRotation(int r, void *);
	static void onMouse(int event, int x, int y, int, void*);
	void receiveSlamMap(nav_msgs::OccupancyGridPtr map);
	void receiveKnownMap(nav_msgs::OccupancyGridPtr map);

	MouseClick waitForMouseClick();
	void assertInBounds(MouseClick mc, cv::Rect roi);

	bool slamMapReceived;
	bool knownMapReceived;
	bool mouseEventReceived;
	MouseClick lastClick;
	int rotation;
	cv::Mat slamMap;
	float slamMapResolution;
	cv::Mat knownMap;
	float knownMapResolution;
	cv::Mat bothMaps;
	std::string windowName;
	ros::Subscriber slamMapSubscriber;
	ros::Subscriber knownMapSubscriber;
};

} /* namespace known_map_localization */

#endif /* SCALE_INVESTIGATOR_SRC_SCALEINVESTIGATOR_H_ */
