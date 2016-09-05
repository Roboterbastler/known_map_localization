/*
 * slam_scale_manager_test.cpp
 *
 *  Created on: 04.09.2016
 *      Author: jacob
 */

#include <gtest/gtest.h>

#include <SlamScaleManager.h>

using namespace known_map_localization;
using namespace geodesy;
using namespace geometry_msgs;

class SlamScaleManager : public ::testing::Test {
};

TEST_F(SlamScaleManager, filtersPositionData) {
	std::vector<UTMPoint> gpsPoints;
	std::vector<Point> slamPoints;
	Point p;

	gpsPoints.push_back(UTMPoint(0, 0, 33, 'U'));
	p.x = 0;
	p.y = 0;
	slamPoints.push_back(p);

	gpsPoints.push_back(UTMPoint(2, 15, 33, 'U'));
	p.x = 1;
	p.y = 9;
	slamPoints.push_back(p);

	gpsPoints.push_back(UTMPoint(2, 15, 32, 'U'));
	p.x = 7;
	p.y = 2;
	slamPoints.push_back(p);

	gpsPoints.push_back(UTMPoint(2, 15, 33, 'U'));
	p.x = 12;
	p.y = 12;
	slamPoints.push_back(p);

	gpsPoints.push_back(UTMPoint(2, 15, 33, 'V'));
	p.x = 11;
	p.y = 23;
	slamPoints.push_back(p);

	UTMPoint gpsPoint(1, 7, 33, 'U');

	slam_scale_manager::SlamScaleManager::filterPositionData(gpsPoints, slamPoints, gpsPoint);

	EXPECT_EQ(3, gpsPoints.size());
	EXPECT_EQ(3, slamPoints.size());
}
