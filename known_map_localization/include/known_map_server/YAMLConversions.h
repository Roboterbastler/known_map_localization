/*
 * YAMLConversions.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_YAMLCONVERSIONS_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_YAMLCONVERSIONS_H_

#include <yaml-cpp/yaml.h>

#include <geographic_msgs/GeoPose.h>

namespace YAML {
template<>
struct convert<geographic_msgs::GeoPose> {
	static Node encode(const geographic_msgs::GeoPose& gp) {
		Node node, position, orientation;
		position["altitude"] = gp.position.altitude;
		position["longitude"] = gp.position.longitude;
		position["latitude"] = gp.position.latitude;
		orientation["x"] = gp.orientation.x;
		orientation["y"] = gp.orientation.y;
		orientation["z"] = gp.orientation.z;
		orientation["w"] = gp.orientation.w;
		node["position"] = position;
		node["orientation"] = orientation;
		return node;
	}

	static bool decode(const Node& node, geographic_msgs::GeoPose& gp) {
		if(!node.IsMap() || node.size() != 2 || !node["position"].IsMap() || !node["orientation"].IsMap()) {
			std::cerr << "Invalid input" << node.IsMap() << node.size() << std::endl;
			return false;
		}

		try {
			gp.position.altitude = node["position"]["altitude"].as<double>();
			gp.position.latitude = node["position"]["latitude"].as<double>();
			gp.position.longitude = node["position"]["longitude"].as<double>();

			gp.orientation.x = node["orientation"]["x"].as<double>();
			gp.orientation.y = node["orientation"]["y"].as<double>();
			gp.orientation.z = node["orientation"]["z"].as<double>();
			gp.orientation.w = node["orientation"]["w"].as<double>();

			return true;
		} catch (InvalidNode &in) {
			std::cerr << "Invalid node: " << in.what() << std::endl;
			return false;
		}
	}
};
} /* namespace YAML */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_YAMLCONVERSIONS_H_ */