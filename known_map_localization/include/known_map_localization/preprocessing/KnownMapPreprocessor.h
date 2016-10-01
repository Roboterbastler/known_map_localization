/*
 * KnownMapPreprocessor.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_KNOWNMAPPREPROCESSOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_KNOWNMAPPREPROCESSOR_H_

#include "MapPreprocessor.h"

namespace kml {

class KnownMapPreprocessor: public MapPreprocessor {
public:
	KnownMapPreprocessor();
};

typedef boost::shared_ptr<KnownMapPreprocessor> KnownMapPreprocessorPtr;
typedef boost::shared_ptr<KnownMapPreprocessor const> KnownMapPreprocessorConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_PREPROCESSING_KNOWNMAPPREPROCESSOR_H_ */
