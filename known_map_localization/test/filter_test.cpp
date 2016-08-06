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
using namespace filter;

TEST(Filter, isNotReadyAtBeginning) {
	FilterPtr filter(new PassThroughFilter());

	ASSERT_THROW(filter->getAlignment(), AlignmentNotAvailable);
}

TEST(PassThroughFilter, isReadyAfterAdding) {
	FilterPtr filter(new PassThroughFilter());

	filter->addAlignment(alignment::StampedAlignment());

	ASSERT_NO_THROW(filter->getAlignment());
}

TEST(PassThroughFilter, passesAlignment) {
	FilterPtr filter(new PassThroughFilter());

	alignment::StampedAlignment alignment;

	filter->addAlignment(alignment);

	alignment::Alignment filteredAlignment = filter->getAlignment();

	ASSERT_EQ(alignment.from, filteredAlignment.from);
	ASSERT_EQ(alignment.to, filteredAlignment.to);
	ASSERT_EQ(alignment.scale, filteredAlignment.scale);
	ASSERT_EQ(alignment.theta, filteredAlignment.theta);
	ASSERT_EQ(alignment.x, filteredAlignment.x);
	ASSERT_EQ(alignment.y, filteredAlignment.y);
}
