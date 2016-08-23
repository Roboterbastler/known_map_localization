/*
 * SlamMapPreprocessor.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_SLAMMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_SLAMMAPPREPROCESSOR_H_

#include "MapPreprocessor.h"

namespace known_map_localization {
namespace preprocessing {

class SlamMapPreprocessor: public MapPreprocessor {
public:
	SlamMapPreprocessor();
};

typedef boost::shared_ptr<SlamMapPreprocessor> SlamMapPreprocessorPtr;
typedef boost::shared_ptr<SlamMapPreprocessor const> SlamMapPreprocessorConstPtr;
} /* namespace preprocessing */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_SLAMMAPPREPROCESSOR_H_ */
