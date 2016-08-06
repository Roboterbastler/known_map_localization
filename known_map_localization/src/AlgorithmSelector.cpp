/*
 * AlgorithmSelector.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <AlgorithmSelector.h>
#include <Exception.h>
#include <aligning/MapstitchAligner.h>
#include <preprocessing/MapstitchSlamMapPreprocessor.h>
#include <preprocessing/MapstitchKnownMapPreprocessor.h>

namespace known_map_localization {

AlgorithmSelector::AlgorithmSelector() {
	std::string algorithm;
	try {
		ros::NodeHandle().getParam("algorithm", algorithm);
	} catch(ros::InvalidNameException &e) {
		throw AlgorithmNotSpecified("Algorithm parameter is missing");
	}

	switch(getAlgorithm(algorithm)) {
	case MAPSTITCH:
		slamMapPreprocessor = preprocessing::SlamMapPreprocessorPtr(new preprocessing::MapstitchSlamMapPreprocessor());
		knownMapPreprocessor = preprocessing::KnownMapPreprocessorPtr(new preprocessing::MapstitchKnownMapPreprocessor());
		aligner = aligning::AlignerPtr(new aligning::MapstitchAligner());
		break;
	case MAPMERGE:
	case CSMERGE:
	default:
		throw IllegalAlgorithm("Invalid algorithm");
	}

	assert(knownMapPreprocessor);
	assert(slamMapPreprocessor);
	assert(aligner);
}

Algorithm AlgorithmSelector::getAlgorithm(std::string algorithm) {
	if(algorithm == "mapstitch") {
		return MAPSTITCH;
	} else {
		throw IllegalAlgorithm("Invalid algorithm name: " + algorithm);
	}
}

preprocessing::SlamMapPreprocessorPtr AlgorithmSelector::getSlamMapPreprocessor() const {
	return slamMapPreprocessor;
}

preprocessing::KnownMapPreprocessorPtr AlgorithmSelector::getKnownMapPreprocessor() const {
	return knownMapPreprocessor;
}

aligning::AlignerPtr AlgorithmSelector::getAligner() const {
	return aligner;
}

} /* namespace known_map_localization */
