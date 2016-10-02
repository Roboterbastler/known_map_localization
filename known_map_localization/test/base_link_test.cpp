/*
 * base_link_test.cpp
 *
 *  Created on: 20.09.2016
 *      Author: jacob
 */

#include <gtest/gtest.h>

#include <base_link/BaseLinkPublisher.h>

TEST(BaseLinkPublisher, orientationToOrientationAngleWithItself) {
	tf::Quaternion rot1 = tf::Quaternion::getIdentity();
	tf::Quaternion rot2;
	rot2.setRPY(0, 0, M_PI);

	// angle with itself = 0?
	EXPECT_FLOAT_EQ(0, kml::BaseLinkPublisher::orientationToOrientationAngle(rot1, rot1));
	EXPECT_FLOAT_EQ(0, kml::BaseLinkPublisher::orientationToOrientationAngle(rot2, rot2));
}

TEST(BaseLinkPublisher, orientationToOrientationAngleOrderDoesNotMatter) {
	tf::Quaternion rot1 = tf::Quaternion::getIdentity();
	tf::Quaternion rot2;
	rot2.setRPY(0, 0, M_PI);

	// angle = 180°, order of orientation does not matter?
	EXPECT_FLOAT_EQ(180, kml::BaseLinkPublisher::orientationToOrientationAngle(rot1, rot2));
	EXPECT_FLOAT_EQ(180, kml::BaseLinkPublisher::orientationToOrientationAngle(rot2, rot1));

	rot2.setRPY(0, 0, M_PI * 1.5);

	// angle = 90°, order of orientation does not matter?
	EXPECT_FLOAT_EQ(90, kml::BaseLinkPublisher::orientationToOrientationAngle(rot1, rot2));
	EXPECT_FLOAT_EQ(90, kml::BaseLinkPublisher::orientationToOrientationAngle(rot2, rot1));
}

TEST(BaseLinkPublisher, orientationToOrientationAngleOtherAngles) {
	tf::Quaternion rot1 = tf::Quaternion::getIdentity();
	tf::Quaternion rot2;
	rot2.setRPY(1.432, 0.534, M_PI / 2);

	// other angles don't interfere, angle = 90°, order of orientation does not matter?
	EXPECT_FLOAT_EQ(90, kml::BaseLinkPublisher::orientationToOrientationAngle(rot1, rot2));
	EXPECT_FLOAT_EQ(90, kml::BaseLinkPublisher::orientationToOrientationAngle(rot2, rot1));
}

TEST(BaseLinkPublisher, poseToPoseAbsDistance) {
	tf::Pose p1(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0));
	tf::Pose p2(tf::Quaternion::getIdentity(), tf::Vector3(1, 0, 0));

	EXPECT_FLOAT_EQ(1, kml::BaseLinkPublisher::poseToPoseAbsDistance(p1, p2));
	EXPECT_FLOAT_EQ(1, kml::BaseLinkPublisher::poseToPoseAbsDistance(p2, p1));

	p2.setOrigin(tf::Vector3(4, 3, 13));

	EXPECT_FLOAT_EQ(5, kml::BaseLinkPublisher::poseToPoseAbsDistance(p1, p2));
	EXPECT_FLOAT_EQ(5, kml::BaseLinkPublisher::poseToPoseAbsDistance(p2, p1));
}

TEST(BaseLinkPublisher, poseToPoseAbsDistanceIgnoreZ) {
	tf::Pose p1(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0));
	tf::Pose p2(tf::Quaternion::getIdentity(), tf::Vector3(4, 3, 13));

	EXPECT_FLOAT_EQ(5, kml::BaseLinkPublisher::poseToPoseAbsDistance(p1, p2));
	EXPECT_FLOAT_EQ(5, kml::BaseLinkPublisher::poseToPoseAbsDistance(p2, p1));
}
