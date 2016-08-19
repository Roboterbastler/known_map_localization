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

bool MapmergeAligner::alignMrgs(nav_msgs::OccupancyGridConstPtr map1, nav_msgs::OccupancyGridConstPtr map2, HypothesesVect &hyp) {
	mrgs_alignment::alignRequest req;
	mrgs_alignment::alignResponse res;

	req.map1 = *map1;
	req.map2 = *map2;
	req.number_of_hypotheses = numberOfHypotheses;

	if(!client.call(req, res)) {
		ROS_ERROR("Align service client failed");
		return false;
	}

	ROS_INFO("Align service call finished in %.2f sec.", res.duration.toSec());

	hyp.clear();
	assert(res.transforms.size() == res.success_coefficients.size());
	for(int i = 0; i < res.transforms.size(); i++) {
		Hypothesis h;
		h.from = map1->header.frame_id;
		h.to = map2->header.frame_id;
		h.scale = 1.;
		h.score = res.success_coefficients.at(i);
		h.stamp = ros::Time::now();

		tf::Quaternion q;
		tf::quaternionMsgToTF(res.transforms.at(i).transform.rotation, q);
		h.theta = quaternionToYawRad(q);
		h.x = res.transforms.at(i).transform.translation.x;
		h.y = res.transforms.at(i).transform.translation.y;
		hyp.push_back(h);
	}

	return true;
}

bool MapmergeAligner::alignMapmerge(nav_msgs::OccupancyGridConstPtr map1, nav_msgs::OccupancyGridConstPtr map2, HypothesesVect &res)
{
  /// Start counting time
  // This is so we can report how long it took to complete the alignment, and adjust the number of hypotheses.
  ros::Time init = ros::Time::now();
  ROS_INFO("Received an alignment request. Alignment initiated.");
  ROS_DEBUG("Dimensions (h x w): %dx%d and %dx%d.", map1->info.height, map1->info.width,
                                                   map2->info.height, map2->info.width);

  /// Calculate dimensioning padding
  // This ensures that both maps get input into mapmerge with the same dimensions
  int map_final_r = 0, map_final_c = 0;

  if(map1->info.height == map2->info.height && map1->info.width == map2->info.width)
  {
    ROS_DEBUG("Maps have equal dimensions, only need to apply rotational padding.");
    map_final_r = map1->info.height;
    map_final_c = map1->info.width;
  }
  else
  {
    ROS_DEBUG("Map dimensions are different. Padding is needed.");
    // Determine which operations to apply
    if(map1->info.height != map2->info.height)
    {
      // Different heights, need to pad
      if(map1->info.height > map2->info.height)
      {
        // Add height to map2 until its height is the same as map1
        map_final_r = map1->info.height;
      }
      else
      {
        // Add height to map1 until its height is the same as map2
        map_final_r = map2->info.height;
      }
    }
    else
    {
      map_final_r = map1->info.height;
    }
    if(map1->info.width != map2->info.width)
    {
      // Different widths, need to pad
      if(map1->info.width > map2->info.width)
      {
        // Add width to map2 until its width is the same as map1
        map_final_c = map1->info.width;
      }
      else
      {
        // Add width to map1 until its width is the same as map2
        map_final_c = map2->info.width;
      }
    }
    else
    {
      map_final_c = map1->info.width;
    }
  }

  // Now we add rotational padding
  // Padding prevents us from losing parts of the map when rotating and translating the maps.
  // We add padding by adding to the final dimension of the map before copying them into mapmerge datatypes.
  // This step is necessary since we are assuming all the maps we receive have been cropped by the map dam node.
  int maximum_distance = ceil(sqrt((pow(map_final_r/2.0,2))+(pow(map_final_c/2.0,2))));
  int padding_rows = maximum_distance - ceil(map_final_r/2.0);
  int padding_cols = maximum_distance - ceil(map_final_c/2.0);
  if(padding_rows < 0) padding_rows = 0;
  if(padding_cols < 0) padding_cols = 0;
  map_final_r += 2*padding_rows;
  map_final_c += 2*padding_cols;

  /// Copy maps into mapmerge datatypes
  // Transfer grid info into datatypes mapmerge can interpret
  // We'll assume values are either -1 for unknown, > MRGS_HIGH_PROB_THRESH for occupied, and < MRGS_LOW_PROB_THRES for
  // free. Check beginning of code.
  // Different values will be classified as unknown.
  ROS_DEBUG("Final dimensions: rows=%d, cols=%d. Copying into mapmerge data.", map_final_r, map_final_c);
  mapmerge::grid_map temp_a(map_final_r, map_final_c),temp_b(map_final_r, map_final_c);
  bool exists_occupied1 = false, exists_occupied2 = false;
  int k = 0, k1 = 0, k2 = 0;  // Linear counters
  for(int i = 0; i < map_final_r; i++)
  {
    for(int j = 0; j < map_final_c;j++)
    {
      // Determine which value to attribute to the current cell
      // (and attribute it)
      if(i < map1->info.height && j < map1->info.width)
      {
        // If (i,j) are inside the original dimensions
        if(map1->data.at(k1) == -1)
          temp_a.grid.at(i).at(j) = 127;
        else if(map1->data.at(k1) < MRGS_LOW_PROB_THRESH)
          temp_a.grid.at(i).at(j) = 255;
        else if (map1->data.at(k1) > MRGS_HIGH_PROB_THRESH)
          {
            temp_a.grid.at(i).at(j) = 0;
            if(!exists_occupied1) exists_occupied1 = true;
          }
          else
            temp_a.grid.at(i).at(j) = 127;
        // Increment linear counter
        k1++;
      }
      else
      {
        // Insert an unknown cell
        temp_a.grid.at(i).at(j) = 127;
      }

      if(i < map2->info.height && j < map2->info.width)
      {
        // If (i,j) are inside the original dimensions
        if(map2->data.at(k2) == -1)
          temp_b.grid.at(i).at(j) = 127;
        else if(map2->data.at(k2) < MRGS_LOW_PROB_THRESH)
          temp_b.grid.at(i).at(j) = 255;
        else if (map2->data.at(k2) > MRGS_HIGH_PROB_THRESH)
          {
            temp_b.grid.at(i).at(j) = 0;
            if(!exists_occupied2) exists_occupied2 = true;
          }
          else
            temp_b.grid.at(i).at(j) = 127;

        // Increment linear counter
        k2++;
      }
      else
      {
        // Insert an unknown cell
        temp_b.grid.at(i).at(j) = 127;
      }

    }
  }

  // Break if there are no occupied cells
  if(!exists_occupied1 || !exists_occupied2){
    ROS_ERROR("At least one of the provided grids contain no occupied cells. Terminating.");
    return true;
  }

  // Translate grids to re-center:
  mapmerge::grid_map a,b;
  mapmerge::translate_map(a, temp_a, -padding_cols, -padding_rows);
  mapmerge::translate_map(b, temp_b, -padding_cols, -padding_rows);

  /// Call mapmerge to determine the transformations
  ROS_DEBUG("Calculating hypotheses.");
  std::vector<mapmerge::transformation> hyp = mapmerge::get_hypothesis(a, b, numberOfHypotheses, 1, false);


  // Show all hypotheses found
  for(int i = 0; i < numberOfHypotheses; i++)
    ROS_DEBUG("Hypothesis %d: ai=%f x=%d y=%d theta=%d", i, hyp[i].ai, hyp[i].deltax, hyp[i].deltay, hyp[i].rotation);

  // Pack results into response message
  ROS_DEBUG("Packing results into response.");

  for(int i = 0; i < hyp.size(); i++) {
	  const mapmerge::transformation hypothesis = hyp[i];

	  /// Compensate origin translation due to padding
//	  map1->info.origin.position.x -= (padding_cols)*map1->info.resolution;
//	  map1->info.origin.position.y -= (padding_rows)*map1->info.resolution;
//	  map2->info.origin.position.x -= (padding_cols)*map1->info.resolution;
//	  map2->info.origin.position.y -= (padding_rows)*map1->info.resolution;

	  /// Calculate transformations
	  ROS_DEBUG("Calculating and packing transforms.");

	  // From map2 to merged_map
	  // Rotation
	  const float deg_to_rad = 0.01745329251; // = pi/180, precalculated for performance.
	  const float theta = deg_to_rad *  hypothesis.rotation;

	  // Center to center
	  tf::Transform center_to_center;
	  tf::Quaternion rotation;

	  // Rotation
	  center_to_center.setIdentity();
	  rotation.setRPY(0, 0, theta);
	  rotation.normalize();
	  center_to_center.setRotation(rotation);
	  // Translation
	  center_to_center.setOrigin(tf::Vector3(-hypothesis.deltax * map1->info.resolution, -hypothesis.deltay * map1->info.resolution, 0));

	  // Map1 to origin
	  tf::Transform map1_to_origin, map2_to_origin, origin_to_center;
	  tf::quaternionMsgToTF(map1->info.origin.orientation, rotation);
	  map1_to_origin.setRotation(rotation);
	  map1_to_origin.setOrigin(tf::Vector3(map1->info.origin.position.x - (padding_cols)*map1->info.resolution, map1->info.origin.position.y - (padding_rows)*map1->info.resolution, map1->info.origin.position.z));
	  // Map2 to origin
	  tf::quaternionMsgToTF(map2->info.origin.orientation, rotation);
	  map2_to_origin.setRotation(rotation);
	  map2_to_origin.setOrigin(tf::Vector3(map2->info.origin.position.x - (padding_cols)*map1->info.resolution, map2->info.origin.position.y - (padding_rows)*map1->info.resolution, map2->info.origin.position.z));

	  // Origin to center
	  origin_to_center.setIdentity();
	  origin_to_center.setOrigin(tf::Vector3((map_final_c * map1->info.resolution)/2.0, (map_final_r * map1->info.resolution)/2.0, 0));

	  // Map to map
	  tf::Transform map1_to_map2;
	  map1_to_map2.setIdentity();
	  map1_to_map2 *= map1_to_origin;
	  map1_to_map2 *= origin_to_center;
	  map1_to_map2 *= center_to_center;
	  map1_to_map2 *= origin_to_center.inverse();
	  map1_to_map2 *= map2_to_origin.inverse();

	  // Pack into response
	  tf::StampedTransform stamped(map1_to_map2, ros::Time::now(), "foo", "bar");
	  geometry_msgs::TransformStamped transform;
	  tf::transformStampedTFToMsg(stamped, transform);

	  Hypothesis h;
	  h.from = map1->header.frame_id;
		h.to = map2->header.frame_id;
		h.scale = 1.;
		h.score = hypothesis.ai;
		h.stamp = ros::Time::now();
		h.theta = quaternionToYawRad(map1_to_map2.getRotation());
		h.x = map1_to_map2.getOrigin().getX();
		h.y = map1_to_map2.getOrigin().getY();
		res.push_back(h);

	  ROS_INFO("Raw hyp: [x=%.2f y=%.2f]", h.x, h.y);
  }
}

MapmergeAligner::MapmergeAligner() :
		useRandomizedHoughTransform(false), useRobust(false), numberOfHypotheses(4) {
	ros::NodeHandle nh("~");

	nh.getParam("use_randomized_hough_transform", useRandomizedHoughTransform);
	nh.getParam("use_robust_algorithm", useRobust);
	nh.getParam("number_of_hypotheses", numberOfHypotheses);

	client = nh.serviceClient<mrgs_alignment::align>("/known_map_localization/mrgs_align_node/align");
	ROS_INFO("Align service: %s", nh.resolveName("/known_map_localization/mrgs_align_node/align").c_str());
}

HypothesesVect MapmergeAligner::align(nav_msgs::OccupancyGridConstPtr knownMap, nav_msgs::OccupancyGridConstPtr slamMap) {
	ros::WallTime start = ros::WallTime::now();

	HypothesesVect res;
//	alignMapmerge(slamMap, knownMap, res);
//	alignMrgs(slamMap, knownMap, res);
//	return res;

	unsigned int mapWidth = std::max(knownMap->info.width, slamMap->info.width);
	unsigned int mapHeight = std::max(knownMap->info.height, slamMap->info.height);

	mapmerge::grid_map knownGrid(mapHeight, mapWidth);
	mapmerge::grid_map slamGrid(mapHeight, mapWidth);
	copyOccupancyGridToGridMap(knownMap, knownGrid);
	copyOccupancyGridToGridMap(slamMap, slamGrid);

	ROS_INFO("Known grid rows(height)=%d cols(width)=%d", knownGrid.get_rows(), knownGrid.get_cols());
	ROS_INFO("SLAM grid rows(height)=%d cols(width)=%d", slamGrid.get_rows(), slamGrid.get_cols());

	// TODO: check that maps contain at least one occupied cell

	std::vector<mapmerge::transformation> hypotheses;
	if(useRobust) {
		hypotheses = mapmerge::get_hypothesis_robust(slamGrid, knownGrid, numberOfHypotheses, 1, useRandomizedHoughTransform);
	} else {
		hypotheses = mapmerge::get_hypothesis(slamGrid, knownGrid, numberOfHypotheses, 1, useRandomizedHoughTransform);
	}

	HypothesesVect resultHypotheses;

	for(int i = 0; i < hypotheses.size(); i++) {
		Hypothesis hypothesis;
		mapmerge::transformation transformation = hypotheses.at(i);

		ROS_INFO("Raw transformation (#%d): x=%d y=%d alpha=%d score=%.4f", i, transformation.deltax, transformation.deltay, transformation.rotation, transformation.ai);

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
		hypothesis.y = mapToMap.getOrigin().getX();
		hypothesis.x = -mapToMap.getOrigin().getY();
//		hypothesis.x = transformation.deltax * knownMap->info.resolution;
//		hypothesis.y = transformation.deltay * knownMap->info.resolution;

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
