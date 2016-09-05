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

using namespace known_map_localization;

class PassThroughFilter : public ::testing::Test {
protected:
	static void SetUpTestCase() {
		ros::Time::init();
	}
};

TEST(Filter, isNotReadyAtBeginning) {
	filter::FilterPtr filter(new filter::PassThroughFilter());

	ASSERT_THROW(filter->getAlignment(), AlignmentNotAvailable);
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
