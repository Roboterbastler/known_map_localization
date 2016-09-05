/*
 * AlgorithmSelector.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <AlgorithmSelector.h>
#include <Exception.h>

namespace known_map_localization {

Algorithm AlgorithmSelector::getAlgorithm(std::string algorithm) {
	if(algorithm == "mapstitch") {
		return MAPSTITCH;
	} if(algorithm == "mapmerge") {
		return MAPMERGE;
	} if(algorithm == "csmerge") {
		return CSMERGE;
	} else {
		throw IllegalAlgorithm("Invalid algorithm name: '" + algorithm + "'");
	}
}

Algorithm AlgorithmSelector::determineAlgorithm() {
	ros::NodeHandle nh("~");
	std::string algorithm;

	if(!nh.getParam("algorithm", algorithm)) {
		throw AlgorithmNotSpecified("Algorithm parameter is missing");
	}

	return getAlgorithm(algorithm);
}

} /* namespace known_map_localization */
