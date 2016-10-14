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

namespace kml {

using namespace std;

MapmergeAligner::MapmergeAligner() :
		mUseRandomizedHoughTransform_(false), mUseRobustAlgorithm_(false), mNumberOfHypotheses_(4) {
	ros::NodeHandle nh("~");

	nh.getParam("use_randomized_hough_transform", mUseRandomizedHoughTransform_);
	nh.getParam("use_robust_algorithm", mUseRobustAlgorithm_);
	nh.getParam("number_of_hypotheses", mNumberOfHypotheses_);
}

HypothesesVect MapmergeAligner::align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap) {
	ros::WallTime start = ros::WallTime::now();

	HypothesesVect resultHypotheses;

	// ensure maps have same dimensions
	unsigned int mapWidth = max(knownMap->info.width, slamMap->info.width);
	unsigned int mapHeight = max(knownMap->info.height, slamMap->info.height);

	// add rotational padding, so no content gets lost when rotating
	unsigned int rotationRadius = ceil(sqrt(pow(mapWidth / 2.0, 2) + pow(mapHeight / 2.0, 2)));
	int paddingWidth = rotationRadius - ceil(mapWidth / 2.0);
	int paddingHeight = rotationRadius - ceil(mapHeight / 2.0);
	paddingWidth = max(0, paddingWidth);
	paddingHeight = max(0, paddingHeight);
	mapWidth += 2 * paddingWidth;
	mapHeight += 2 * paddingHeight;

	mapmerge::grid_map knownGridTmp(mapHeight, mapWidth);
	mapmerge::grid_map slamGridTmp(mapHeight, mapWidth);
	mapmerge::grid_map knownGrid, slamGrid;

	// copy maps and check for empty maps
	bool mapsNonEmpty = true;
	mapsNonEmpty = mapsNonEmpty && copyOccupancyGridToGridMap(knownMap, knownGridTmp);
	mapsNonEmpty = mapsNonEmpty && copyOccupancyGridToGridMap(slamMap, slamGridTmp);

	// check that maps contain at least one occupied cell
	if(!mapsNonEmpty) {
		ROS_WARN("One of the maps to align does not contain at least one occupied cell.");
		return resultHypotheses;
	}

	// re-center maps
	mapmerge::translate_map(knownGrid, knownGridTmp, -paddingWidth, -paddingHeight);
	mapmerge::translate_map(slamGrid, slamGridTmp, -paddingWidth, -paddingHeight);

	// compute hypotheses
	std::vector<mapmerge::transformation> hypotheses;
	if(mUseRobustAlgorithm_) {
		hypotheses = mapmerge::get_hypothesis_robust(slamGrid, knownGrid, mNumberOfHypotheses_, 1, mUseRandomizedHoughTransform_);
	} else {
		hypotheses = mapmerge::get_hypothesis(slamGrid, knownGrid, mNumberOfHypotheses_, 1, mUseRandomizedHoughTransform_);
	}

	tf::Quaternion rotation;
	float resolution = knownMap->info.resolution;

	// padding offset
	tf::Transform paddedOriginToMapOrigin;
	paddedOriginToMapOrigin.setIdentity();
	paddedOriginToMapOrigin.setOrigin(tf::Vector3(paddingWidth * resolution, paddingHeight * resolution, 0));

	// known map frame to known map origin
	tf::Transform knownMapFrameToOrigin;
	knownMapFrameToOrigin.setIdentity();
	knownMapFrameToOrigin.setOrigin(tf::Vector3(knownMap->info.origin.position.x, knownMap->info.origin.position.y, 0));
	tf::quaternionMsgToTF(knownMap->info.origin.orientation, rotation);
	knownMapFrameToOrigin.setRotation(rotation);

	// SLAM map frame to SLAM map origin
	tf::Transform slamMapFrameToOrigin;
	slamMapFrameToOrigin.setIdentity();
	slamMapFrameToOrigin.setOrigin(tf::Vector3(slamMap->info.origin.position.x, slamMap->info.origin.position.y, 0));
	tf::quaternionMsgToTF(slamMap->info.origin.orientation, rotation);
	slamMapFrameToOrigin.setRotation(rotation);

	// get transformations from hypotheses
	for(int i = 0; i < hypotheses.size(); i++) {
		Hypothesis hypothesis;
		mapmerge::transformation transformation = hypotheses.at(i);

		// map alignment computed by mapmerge
		tf::Transform slamOriginToKnownOrigin;
		slamOriginToKnownOrigin.setIdentity();
		slamOriginToKnownOrigin.setOrigin(tf::Vector3(resolution * transformation.deltax, resolution * transformation.deltay, 0));
		rotation.setRPY(0, 0, degToRad(transformation.rotation));
		slamOriginToKnownOrigin.setRotation(rotation);

		// SLAM map frame to known map frame
		tf::Transform mapFrameToMapFrame;
		mapFrameToMapFrame.setIdentity();
		mapFrameToMapFrame *= slamMapFrameToOrigin;
		mapFrameToMapFrame *= paddedOriginToMapOrigin.inverse();
		mapFrameToMapFrame *= slamOriginToKnownOrigin;
		mapFrameToMapFrame *= paddedOriginToMapOrigin;
		mapFrameToMapFrame *= knownMapFrameToOrigin.inverse();

		hypothesis.from = slamMap->header.frame_id;
		hypothesis.to = knownMap->header.frame_id;
		hypothesis.scale = 1.;
		hypothesis.score = transformation.ai;
		hypothesis.stamp = ros::Time::now();
		hypothesis.theta = tf::getYaw(mapFrameToMapFrame.getRotation());
		hypothesis.x = mapFrameToMapFrame.getOrigin().getX();
		hypothesis.y = mapFrameToMapFrame.getOrigin().getY();

		resultHypotheses.push_back(hypothesis);
	}

	ros::WallDuration duration = ros::WallTime::now() - start;
	ROS_INFO("Successfully completed aligning in %f sec", duration.toSec());

	return resultHypotheses;
}

bool copyOccupancyGridToGridMap(nav_msgs::OccupancyGridConstPtr occGrid, mapmerge::grid_map &gridMap) {
	ROS_ASSERT(occGrid);
	ROS_ASSERT(gridMap.get_cols() >= occGrid->info.width);
	ROS_ASSERT(gridMap.get_rows() >= occGrid->info.height);

	if(occGrid->info.width == 0 || occGrid->info.height == 0) {
		ROS_WARN("Tried to copy map with 0 area.");
		return false;
	}

	nav_msgs::OccupancyGrid::_data_type::const_iterator occCellIt = occGrid->data.begin();
	std::vector<std::vector<unsigned int> >::iterator rowIt;
	std::vector<unsigned int>::iterator gridCellIt;
	bool containsOccupiedCell = false;

	// fill all cells with 'unknown' value
	for(rowIt = gridMap.grid.begin(); rowIt != gridMap.grid.end(); ++rowIt) {
		for(gridCellIt = rowIt->begin(); gridCellIt != rowIt->end(); ++gridCellIt) {
			*gridCellIt = gridMap.get_unknown_cell();
		}
	}

	// copy 'occupied' and 'free' cells from occupancy grid
	rowIt = gridMap.grid.begin();

	for(int row = 0; row < occGrid->info.height; row++, ++rowIt) {
		gridCellIt = rowIt->begin();
		for(int col = 0; col < occGrid->info.width; col++, ++gridCellIt, ++occCellIt) {
			if(*occCellIt == -1) {
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

} /* namespace kml */
