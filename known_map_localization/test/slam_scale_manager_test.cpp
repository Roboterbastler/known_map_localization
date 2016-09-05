/*
 * slam_scale_manager_test.cpp
 *
 *  Created on: 04.09.2016
 *      Author: jacob
 */

#include <gtest/gtest.h>

#include <SlamScaleManager.h>

using namespace known_map_localization::slam_scale_manager;
using namespace geodesy;
using namespace geometry_msgs;

TEST(PositionUTMZoneFilter, classifiesCorrectly) {
	PositionUTMZoneFilter filter(33, 'U');

	EXPECT_FALSE(filter(PositionPair(UTMPoint(0, 0, 33, 'U'), Point())));
	EXPECT_FALSE(filter(PositionPair(UTMPoint(0, 1, 33, 'U'), Point())));
	EXPECT_FALSE(filter(PositionPair(UTMPoint(-5, 0, 33, 'U'), Point())));

	EXPECT_TRUE(filter(PositionPair(UTMPoint(0, 0, 32, 'U'), Point())));
	EXPECT_TRUE(filter(PositionPair(UTMPoint(0, 0, 33, 'V'), Point())));
	EXPECT_TRUE(filter(PositionPair(UTMPoint(0, 0, 15, 'S'), Point())));
}

TEST(SlamScaleManager, geometryPointDistance) {
	Point p1, p2;
	p1.x = 0;
	p1.y = 0;
	p2.x = 0;
	p2.y = 0;

	EXPECT_FLOAT_EQ(0., SlamScaleManager::distance(p1, p2));

	p2.x = 1;

	EXPECT_FLOAT_EQ(1., SlamScaleManager::distance(p1, p2));

	p2.y = 1;

	EXPECT_FLOAT_EQ(sqrt(2), SlamScaleManager::distance(p1, p2));
}

TEST(SlamScaleManager, utmPointDistance) {
	UTMPoint p1, p2;
	p1.northing = 0;
	p1.easting = 0;
	p2.northing = 0;
	p2.easting = 0;

	EXPECT_FLOAT_EQ(0., SlamScaleManager::distance(p1, p2));

	p2.northing = 1;

	EXPECT_FLOAT_EQ(1., SlamScaleManager::distance(p1, p2));

	p2.easting = 1;

	EXPECT_FLOAT_EQ(sqrt(2), SlamScaleManager::distance(p1, p2));
}
