/*
 * KnownMapServer.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <libgen.h>
#include <yaml-cpp/yaml.h>

#include <nav_msgs/GetMap.h>
#include <map_server/image_loader.h>

#include "YAMLConversions.h"
#include "KnownMapServer.h"

namespace known_map_localization {
namespace known_map_server {
using namespace preprocessing;

KnownMapServer::KnownMapServer(KnownMapPreprocessorPtr preprocessor) :
		preprocessor(preprocessor) {
	if(ros::isInitialized()) {
		ros::NodeHandle nh;
		knownMapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("known_map", 4, true);

		std::string fileName;
		nh.getParam("known_map_config_file", fileName);

		ROS_INFO("Load known map...");
		if(!loadKnownMap(fileName)) {
			ROS_ERROR("Known map could not be loaded. It will not be published at all.");
			return;
		}

		ROS_INFO("Preprocessing of known map...");
		preprocessor->process(knownMap);

		knownMapPublisher.publish(knownMap);
	}
}

bool KnownMapServer::loadKnownMap(std::string fileName) {
	YAML::Node mapMetaData;
	try {
		mapMetaData = YAML::LoadFile(fileName);
	} catch (YAML::BadFile &bf) {
		ROS_ERROR_STREAM("Bad YAML file, could not load map meta data! " << bf.what());
		return false;
	}

	if(!knownMapAnchor) {
		knownMapAnchor = geographic_msgs::GeoPosePtr(new geographic_msgs::GeoPose());
	}
	if(!knownMap) {
		knownMap = nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid());
	}

	double freeThresh, occupiedTresh, resolution, yaw;
	double origin[3] = {0, 0, 0};
	bool negate;
	std::string mapFileName;

	try {
		*knownMapAnchor = mapMetaData["anchor"].as<geographic_msgs::GeoPose>();
		freeThresh = mapMetaData["free_thresh"].as<double>();
		occupiedTresh = mapMetaData["occupied_thresh"].as<double>();
		resolution = mapMetaData["resolution"].as<double>();
		negate = mapMetaData["negate"].as<int>() > 0;
		mapFileName = mapMetaData["image"].as<std::string>();
		origin[0] = mapMetaData["origin"][0].as<double>();
		origin[1] = mapMetaData["origin"][1].as<double>();
		yaw = mapMetaData["origin"][2].as<double>();
	} catch(YAML::InvalidNode &in) {
		ROS_ERROR_STREAM("Invalid YAML file, could not load map meta data! " << in.what());
		return false;
	}

	nav_msgs::GetMapResponse mapResp;

	if(mapFileName[0] != '/') {
		// get absolute map file name
		mapFileName = absoluteMapFileName(mapFileName, fileName);
	}

	ROS_DEBUG("Load known map from: %s", mapFileName.c_str());

	try {
		map_server::loadMapFromFile(&mapResp, mapFileName.c_str(), resolution, negate, occupiedTresh, freeThresh, origin, TRINARY);
	} catch(std::runtime_error &e) {
		ROS_ERROR_STREAM("Bad map file, could not load map! " << e.what());
		return false;
	}

	*knownMap = mapResp.map;
	ros::NodeHandle().getParam("known_map_frame_id", knownMap->header.frame_id);
	knownMap->info.origin.orientation = tf::createQuaternionMsgFromYaw(yaw);

	return true;
}

std::string KnownMapServer::absoluteMapFileName(std::string mapFileName, std::string configFileName) {
	char *configFileNameCopy = strdup(configFileName.c_str());
	std::string absMapFileName = std::string(dirname(configFileNameCopy)) + '/' + mapFileName;
	free(configFileNameCopy);
	return absMapFileName;
}

} /* namespace known_map_server */
} /* namespace known_map_localization */
