/*
 * filter_test.cpp
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#include <gtest/gtest.h>

#include <Exception.h>
#include <filter/Filter.h>
#include <filter/PassThroughFilter.h>
#include <filter/GpsFilter.h>

using namespace known_map_localization;

class Filter : public ::testing::Test {
protected:
	static void SetUpTestCase() {
		ros::Time::init();
		ros::console::shutdown();
	}
};

//class PassThroughFilter : public ::testing::Test {
//protected:
//	static void SetUpTestCase() {
//		ros::Time::init();
//	}
//};

class GpsFilter : public ::testing::Test {
protected:
	static void SetUpTestCase() {
		ros::Time::init();
	}
};

TEST_F(Filter, isNotReadyAtBeginning) {
	filter::FilterPtr filter(new filter::PassThroughFilter());

	ASSERT_THROW(filter->getAlignment(), AlignmentNotAvailable);
}

TEST_F(GpsFilter, convertGPSPositionToAnchorFrameDifferentZones) {
	geographic_msgs::GeoPose anchor;
	anchor.position.latitude = 52.51273962;
	anchor.position.longitude = 13.32486415;
	tf::Quaternion anchorHeading = tf::Quaternion::getIdentity();
	tf::quaternionTFToMsg(anchorHeading, anchor.orientation);

	sensor_msgs::NavSatFix fix2;
	fix2.latitude = 48.86421235;
	fix2.longitude = 2.32709909;

	EXPECT_THROW(filter::GpsFilter::convertGPSPositionToAnchorFrame(fix2, anchor), DifferentUTMGridZones);
}

TEST_F(GpsFilter, convertGPSPositionToAnchorFrameNoRotation) {
	geographic_msgs::GeoPose anchor;
	anchor.position.latitude = 52.51273962;
	anchor.position.longitude = 13.32486415;
	tf::Quaternion anchorHeading = tf::Quaternion::getIdentity();
	tf::quaternionTFToMsg(anchorHeading, anchor.orientation);

	sensor_msgs::NavSatFix fix1;
	fix1.latitude = 52.51145984;
	fix1.longitude = 13.32782531;
	fix1.header.stamp = ros::Time::now();

	geometry_msgs::PointStamped result;
	ASSERT_NO_THROW(result = filter::GpsFilter::convertGPSPositionToAnchorFrame(fix1, anchor));

	EXPECT_EQ(fix1.header.stamp, result.header.stamp);
	EXPECT_NEAR(197.6, result.point.x, 0.1);
	EXPECT_NEAR(-147, result.point.y, 0.1);
	EXPECT_EQ(0, result.point.z);
}

TEST_F(GpsFilter, convertGPSPositionToAnchorFrameRotation) {
	geographic_msgs::GeoPose anchor;
	anchor.position.latitude = 52.51273962;
	anchor.position.longitude = 13.32486415;
	tf::Quaternion anchorHeading;

	sensor_msgs::NavSatFix fix1;
	fix1.latitude = 52.51145984;
	fix1.longitude = 13.32782531;
	fix1.header.stamp = ros::Time::now();

	anchorHeading.setRPY(0, 0, M_PI / 2.);
	tf::quaternionTFToMsg(anchorHeading, anchor.orientation);

	geometry_msgs::PointStamped result = filter::GpsFilter::convertGPSPositionToAnchorFrame(fix1, anchor);
	EXPECT_NEAR(-147, result.point.x, 0.1);
	EXPECT_NEAR(-197.6, result.point.y, 0.1);
	EXPECT_EQ(0, result.point.z);
}

//TEST_F(PassThroughFilter, isReadyAfterAdding) {
//	filter::FilterPtr filter(new filter::PassThroughFilter());
//
//	filter->addHypotheses(alignment::HypothesesVect(1, alignment::StampedAlignment()));
//
//	ASSERT_NO_THROW(filter->getAlignment());
//}
//
//TEST_F(PassThroughFilter, passesAlignment) {
//	filter::FilterPtr filter(new filter::PassThroughFilter());
//
//	alignment::StampedAlignment alignment;
//	alignment.from = "origin_frame";
//	alignment.to = "target_frame";
//	alignment.theta = M_PI / 4;
//	alignment.scale = 0.87;
//	alignment.x = 141.9;
//	alignment.y = -32.4;
//
//	filter->addHypotheses(alignment::HypothesesVect(1, alignment));
//
//	alignment::Alignment filteredAlignment = filter->getAlignment();
//
//	ASSERT_EQ(alignment.from, filteredAlignment.from);
//	ASSERT_EQ(alignment.to, filteredAlignment.to);
//	ASSERT_EQ(alignment.scale, filteredAlignment.scale);
//	ASSERT_EQ(alignment.theta, filteredAlignment.theta);
//	ASSERT_EQ(alignment.x, filteredAlignment.x);
//	ASSERT_EQ(alignment.y, filteredAlignment.y);
//}
