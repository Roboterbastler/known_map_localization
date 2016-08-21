/*
 * MapmergeAligner.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <mapmerge/manipulatemap.h>
#include <mrgs_alignment/align.h>

#include <aligning/MapmergeAligner.h>
#include <Utils.h>

#define OCCUPIED_THRESHOLD 80
#define FREE_THRESHOLD 20

#define MRGS_LOW_PROB_THRESH 10
#define MRGS_HIGH_PROB_THRESH 70

namespace known_map_localization {
namespace aligning {

using namespace alignment;

MapmergeAligner::MapmergeAligner() :
		useRandomizedHoughTransform(false), useRobust(false), numberOfHypotheses(4) {
	ros::NodeHandle nh("~");

	nh.getParam("use_randomized_hough_transform", useRandomizedHoughTransform);
	nh.getParam("use_robust_algorithm", useRobust);
	nh.getParam("number_of_hypotheses", numberOfHypotheses);
}

HypothesesVect MapmergeAligner::align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap) {
	ros::WallTime start = ros::WallTime::now();

	HypothesesVect resultHypotheses;

	// copy maps to grid maps and ensure they have same dimensions
	unsigned int mapWidth = std::max(knownMap->info.width, slamMap->info.width);
	unsigned int mapHeight = std::max(knownMap->info.height, slamMap->info.height);
	bool mapsNonEmpty = true;

	mapmerge::grid_map knownGrid(mapHeight, mapWidth);
	mapmerge::grid_map slamGrid(mapHeight, mapWidth);
	mapsNonEmpty = mapsNonEmpty && copyOccupancyGridToGridMap(knownMap, knownGrid);
	mapsNonEmpty = mapsNonEmpty && copyOccupancyGridToGridMap(slamMap, slamGrid);

	// check that maps contain at least one occupied cell
	if(!mapsNonEmpty) {
		ROS_WARN("One of the maps to align does not contain at least one occupied cell.");
		return resultHypotheses;
	}

	// compute hypotheses
	std::vector<mapmerge::transformation> hypotheses;
	if(useRobust) {
		hypotheses = mapmerge::get_hypothesis_robust(slamGrid, knownGrid, numberOfHypotheses, 1, useRandomizedHoughTransform);
	} else {
		hypotheses = mapmerge::get_hypothesis(slamGrid, knownGrid, numberOfHypotheses, 1, useRandomizedHoughTransform);
	}

	// get transformations from hypotheses
	for(int i = 0; i < hypotheses.size(); i++) {
		Hypothesis hypothesis;
		mapmerge::transformation transformation = hypotheses.at(i);

		tf::Transform centerToCenter;
		tf::Quaternion rotation;
		centerToCenter.setIdentity();
		centerToCenter.setOrigin(tf::Vector3(knownMap->info.resolution*transformation.deltax, knownMap->info.resolution*transformation.deltay, 0));
		rotation.setRPY(0, 0, degToRad(transformation.rotation));
		centerToCenter.setRotation(rotation);
		tf::Transform mapOriginToCenter;
		mapOriginToCenter.setIdentity();
		mapOriginToCenter.setOrigin(tf::Vector3(knownMap->info.resolution*mapWidth / 2., knownMap->info.resolution*mapHeight / 2., 0));

		tf::Transform mapToMap;
		mapToMap.setIdentity();
		mapToMap *= mapOriginToCenter;
		mapToMap *= centerToCenter;
		mapToMap *= mapOriginToCenter.inverse();

		hypothesis.from = slamMap->header.frame_id;
		hypothesis.to = knownMap->header.frame_id;
		hypothesis.scale = 1.;
		hypothesis.score = transformation.ai;
		hypothesis.stamp = ros::Time::now();
		hypothesis.theta = degToRad(transformation.rotation);
		hypothesis.x = transformation.deltax * knownMap->info.resolution;
		hypothesis.y = transformation.deltay * knownMap->info.resolution;

		resultHypotheses.push_back(hypothesis);
	}

	ros::WallDuration duration = ros::WallTime::now() - start;
	ROS_INFO("Successfully completed aligning in %f sec", duration.toSec());

	return resultHypotheses;
}

bool MapmergeAligner::copyOccupancyGridToGridMap(nav_msgs::OccupancyGridConstPtr occGrid, mapmerge::grid_map &gridMap) {
	assert(occGrid);
	assert(gridMap.get_cols() >= occGrid->info.width);
	assert(gridMap.get_rows() >= occGrid->info.height);

	if(occGrid->info.width == 0 || occGrid->info.height == 0) {
		ROS_WARN("Tried to copy map with 0 area.");
		return false;
	}

	nav_msgs::OccupancyGrid::_data_type::const_iterator occCellIt = occGrid->data.begin();
	std::vector<std::vector<unsigned int> >::iterator rowIt = gridMap.grid.begin();
	std::vector<unsigned int>::iterator gridCellIt;
	bool containsOccupiedCell = false;

	for(int row = 0; row < occGrid->info.height; row++, ++rowIt) {
		gridCellIt = rowIt->begin();
		for(int col = 0; col < occGrid->info.width; col++, ++gridCellIt, ++occCellIt) {
			if(*occCellIt == -1) {
				*gridCellIt = gridMap.get_unknown_cell();
				continue;
			}
			if(*occCellIt < FREE_THRESHOLD) {
				*gridCellIt = gridMap.get_free_cell();
				continue;
			}
			if(*occCellIt > OCCUPIED_THRESHOLD) {
				*gridCellIt = gridMap.get_occupied_cell();
				containsOccupiedCell = true;
			}
		}
	}

	return containsOccupiedCell;
}

} /* namespace aligning */
} /* namespace known_map_localization */
