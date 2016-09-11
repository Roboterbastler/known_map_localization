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

TEST(SlamScaleManager, medianEmpty) {
	std::vector<double> empty;

	ASSERT_THROW(SlamScaleManager::median(empty), std::out_of_range);
}

TEST(SlamScaleManager, medianEven) {
	std::vector<double> data;
	data.push_back(6);
	data.push_back(3);

	data.push_back(5);
	data.push_back(10);

	data.push_back(8);
	data.push_back(2);

	data.push_back(5);
	data.push_back(11);

	ASSERT_EQ(5.5, SlamScaleManager::median(data));
}

TEST(SlamScaleManager, medianOdd) {
	std::vector<double> data;
	data.push_back(6);

	data.push_back(5);
	data.push_back(10);

	data.push_back(8);
	data.push_back(2);

	data.push_back(5);
	data.push_back(11);

	ASSERT_EQ(6, SlamScaleManager::median(data));
}
