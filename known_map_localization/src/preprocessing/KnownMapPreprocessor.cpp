/*
 * KnownMapPreprocessor.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <preprocessing/KnownMapPreprocessor.h>
#include <AlgorithmSelector.h>
#include <preprocessing/MapstitchKnownMapPreprocessor.h>
#include <preprocessing/MapmergeKnownMapPreprocessor.h>
#include <Exception.h>

namespace known_map_localization {
namespace preprocessing {

KnownMapPreprocessorPtr KnownMapPreprocessor::_instance;

KnownMapPreprocessor::KnownMapPreprocessor() :
		MapPreprocessor("processed_known_map_img", "known_preprocessing_enabled") {
}

KnownMapPreprocessorPtr KnownMapPreprocessor::instance() {
	if(!_instance) {
		switch(AlgorithmSelector::determineAlgorithm()) {
		case MAPSTITCH:
				_instance = KnownMapPreprocessorPtr(new MapstitchKnownMapPreprocessor());
				break;
			case MAPMERGE:
				_instance = KnownMapPreprocessorPtr(new MapmergeKnownMapPreprocessor());
				break;
			case CSMERGE:
			default:
				throw IllegalAlgorithm("Invalid algorithm");
		}
	}
	return _instance;
}

} /* namespace preprocessing */
} /* namespace known_map_localization */
