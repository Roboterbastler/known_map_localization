/*
 * KnownMapServer.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_SERVER_KNOWNMAPSERVER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_SERVER_KNOWNMAPSERVER_H_

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geographic_msgs/GeoPose.h>

namespace known_map_localization {
namespace known_map_server {

class KnownMapServer;
typedef boost::shared_ptr<KnownMapServer> KnownMapServerPtr;
typedef boost::shared_ptr<KnownMapServer const> KnownMapServerConstPtr;

/**
 * # Known Map Server
 *
 * This module load the known map from disc and publishes it as a latched topic.
 * Preprocessing steps are applied to the map.
 *
 * ## Published Topics
 * - **known_map**: The (preprocessed) known map
 *
 * ## Parameters
 * - **known_map_config_file**: The file name of the YAML configuration
 * file specifying the known map
 * - **known_map_frame_id**: The frame ID of the published known map
 */
class KnownMapServer {
public:
	static KnownMapServerPtr instance();

	/**
	 * Get the known map.
	 * @return Pointer to the known map
	 */
	nav_msgs::OccupancyGridConstPtr getKnownMap() const;

	/**
	 * Get the anchor, which is a geographic_msgs::GeoPose.
	 * @return Pointer to the anchor
	 */
	geographic_msgs::GeoPoseConstPtr getAnchor() const;

protected:
	KnownMapServer();

private:
	static KnownMapServerPtr _instance;

	/**
	 * Loads the known map from disc.
	 * @param fileName File name of the YAML map meta data file
	 * @return Indicates success or failure
	 * @throws YAML::BadFile if the YAML file could not be loaded
	 * @throws YAML::InvalidNode if the YAML file contains invalid data
	 * @throws std::runtime_error if loading the map fails
	 */
	bool loadKnownMap(std::string fileName);

	/**
	 * Retrieves the absolute file name, given the absolute file name of the
	 * configuration (YAML) file to which the map file name is relative to.
	 * @param mapFileName The relative map file name
	 * @param configFileName The absolute configuration file name
	 * @return The absolute map file name
	 */
	static std::string absoluteMapFileName(std::string mapFileName, std::string configFileName);

	/// Publishes the known map via a latched topic
	ros::Publisher knownMapPublisher;

	/// The known map
	nav_msgs::OccupancyGridPtr knownMap;

	/// The anchor of the known map
	geographic_msgs::GeoPosePtr knownMapAnchor;
};

} /* namespace known_map_server */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_SERVER_KNOWNMAPSERVER_H_ */
