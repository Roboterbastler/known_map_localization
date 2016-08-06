/*
 * alignment_test.cpp
 *
 *  Created on: 06.08.2016
 *      Author: jacob
 */

#include <gtest/gtest.h>

#include <alignment/Alignment.h>
#include <alignment/StampedAlignment.h>

using namespace known_map_localization;
using namespace alignment;

TEST(Alignment, givesCorrectIdentity) {
	Alignment a = Alignment::getIdentity();

	ASSERT_EQ(0., a.theta);
	ASSERT_EQ(0., a.x);
	ASSERT_EQ(0., a.y);
	ASSERT_EQ(1., a.scale);
}

TEST(Alignment, convertsToCorrectTfTransform) {
	Alignment id = Alignment::getIdentity();
	tf::Transform tfId = id.toTfTransform();

	ASSERT_FLOAT_EQ(0., tfId.getOrigin().getX());
	ASSERT_FLOAT_EQ(0., tfId.getOrigin().getY());
	ASSERT_FLOAT_EQ(0., tfId.getOrigin().getZ());
	ASSERT_FLOAT_EQ(0., tfId.getRotation().getX());
	ASSERT_FLOAT_EQ(0., tfId.getRotation().getY());
	ASSERT_FLOAT_EQ(0., tfId.getRotation().getZ());
	ASSERT_FLOAT_EQ(1., tfId.getRotation().getW());

	Alignment b;
	b.scale = 1.25;
	b.theta = M_PI / 2;
	b.x = 5.;
	b.y = 3.5;
	tfId = b.toTfTransform();
	tf::Quaternion q;
	q.setRPY(0., 0., b.theta);


	ASSERT_FLOAT_EQ(5., tfId.getOrigin().getX());
	ASSERT_FLOAT_EQ(3.5, tfId.getOrigin().getY());
	ASSERT_FLOAT_EQ(0., tfId.getOrigin().getZ());
	ASSERT_FLOAT_EQ(q.getX(), tfId.getRotation().getX());
	ASSERT_FLOAT_EQ(q.getY(), tfId.getRotation().getY());
	ASSERT_FLOAT_EQ(q.getZ(), tfId.getRotation().getZ());
	ASSERT_FLOAT_EQ(q.getW(), tfId.getRotation().getW());
}

TEST(StampedAlignment, givesCorrectIdentity) {
	StampedAlignment a = StampedAlignment::getIdentity();

	ASSERT_EQ(0., a.theta);
	ASSERT_EQ(0., a.x);
	ASSERT_EQ(0., a.y);
	ASSERT_EQ(1., a.scale);
}
