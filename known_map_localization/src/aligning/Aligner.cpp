/*
 * Aligner.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include <aligning/Aligner.h>
#include <AlgorithmSelector.h>
#include <aligning/MapstitchAligner.h>
#include <aligning/MapmergeAligner.h>
#include <Exception.h>

namespace known_map_localization {
namespace aligning {

AlignerPtr Aligner::_instance;

Aligner::Aligner() {

}

Aligner::~Aligner() {
}

AlignerPtr Aligner::instance() {
	if(!_instance) {
		switch(AlgorithmSelector::determineAlgorithm()) {
		case MAPSTITCH:
			_instance = AlignerPtr(new MapstitchAligner());
			break;
		case MAPMERGE:
			_instance = AlignerPtr(new MapmergeAligner());
			break;
		case CSMERGE:
		default:
			throw IllegalAlgorithm("Invalid algorithm");
		}
	}
	return _instance;
}

} /* namespace aligning */
} /* namespace known_map_localization */
