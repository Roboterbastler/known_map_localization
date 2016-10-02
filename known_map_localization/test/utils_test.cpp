/*
 * utils_test.cpp
 *
 *  Created on: 04.09.2016
 *      Author: jacob
 */

#include <math.h>

#include <gtest/gtest.h>

#include <Utils.h>

TEST(Utils, degToRad) {
	EXPECT_FLOAT_EQ(M_PI, kml::degToRad(180));
	EXPECT_FLOAT_EQ(0, kml::degToRad(0));
	EXPECT_FLOAT_EQ(2 * M_PI, kml::degToRad(360));
	EXPECT_FLOAT_EQ(-M_PI / 2, kml::degToRad(-90));
}

TEST(Utils, radToDeg) {
	EXPECT_FLOAT_EQ(0, kml::radToDeg(0));
	EXPECT_FLOAT_EQ(90, kml::radToDeg(M_PI / 2));
	EXPECT_FLOAT_EQ(180, kml::radToDeg(M_PI));
	EXPECT_FLOAT_EQ(360, kml::radToDeg(2 * M_PI));
	EXPECT_FLOAT_EQ(-180, kml::radToDeg(-M_PI));
}

TEST(Utils, geometryPointDistance) {
	geometry_msgs::Point p1, p2;
	p1.x = 0;
	p1.y = 0;
	p2.x = 0;
	p2.y = 0;

	EXPECT_FLOAT_EQ(0., kml::distance(p1, p2));

	p2.x = 1;

	EXPECT_FLOAT_EQ(1., kml::distance(p1, p2));

	p2.y = 1;

	EXPECT_FLOAT_EQ(sqrt(2), kml::distance(p1, p2));
}

TEST(Utils, medianEmpty) {
	std::vector<double> empty;

	ASSERT_THROW(kml::median(empty), std::out_of_range);
}

TEST(Utils, medianEven) {
	std::vector<double> data;
	data.push_back(6);
	data.push_back(3);

	data.push_back(5);
	data.push_back(10);

	data.push_back(8);
	data.push_back(2);

	data.push_back(5);
	data.push_back(11);

	ASSERT_EQ(5.5, kml::median(data));
}

TEST(Utils, medianOdd) {
	std::vector<double> data;
	data.push_back(6);

	data.push_back(5);
	data.push_back(10);

	data.push_back(8);
	data.push_back(2);

	data.push_back(5);
	data.push_back(11);

	ASSERT_EQ(6, kml::median(data));
}
