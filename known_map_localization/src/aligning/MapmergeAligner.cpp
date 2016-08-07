/*
 * MapmergeAligner.cpp
 *
 *  Created on: 07.08.2016
 *      Author: jacob
 */

#include <ros/ros.h>
#include <mapmerge/manipulatemap.h>

#include <aligning/MapmergeAligner.h>
#include <Utils.h>

#define OCCUPIED_THRESHOLD 80
#define FREE_THRESHOLD 20

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

	unsigned int mapWidth = std::max(knownMap->info.width, slamMap->info.width);
	unsigned int mapHeight = std::max(knownMap->info.height, slamMap->info.height);

	mapmerge::grid_map knownGrid(mapHeight, mapWidth);
	mapmerge::grid_map slamGrid(mapHeight, mapWidth);
	copyOccupancyGridToGridMap(knownMap, knownGrid);
	copyOccupancyGridToGridMap(slamMap, slamGrid);

	std::vector<mapmerge::transformation> hypotheses;
	if(useRobust) {
		hypotheses = mapmerge::get_hypothesis_robust(knownGrid, slamGrid, numberOfHypotheses, 1, useRandomizedHoughTransform);
	} else {
		hypotheses = mapmerge::get_hypothesis(knownGrid, slamGrid, numberOfHypotheses, 1, useRandomizedHoughTransform);
	}

	HypothesesVect resultHypotheses;

	for(int i = 0; i < hypotheses.size(); i++) {
		Hypothesis hypothesis;
		const mapmerge::transformation &transformation = hypotheses.at(i);

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

void MapmergeAligner::copyOccupancyGridToGridMap(nav_msgs::OccupancyGridConstPtr occGrid, mapmerge::grid_map &gridMap) {
	assert(occGrid);
	assert(gridMap.get_cols() >= occGrid->info.width);
	assert(gridMap.get_rows() >= occGrid->info.height);

	if(occGrid->info.width == 0 || occGrid->info.height == 0) {
		ROS_WARN("Tried to copy map with 0 area.");
		return;
	}

	nav_msgs::OccupancyGrid::_data_type::const_iterator occCellIt = occGrid->data.begin();
	std::vector<std::vector<unsigned int> >::iterator rowIt = gridMap.grid.begin();
	std::vector<unsigned int>::iterator gridCellIt;

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
			}
		}
	}
}

} /* namespace aligning */
} /* namespace known_map_localization */
