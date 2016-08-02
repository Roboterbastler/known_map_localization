/*
 * KnownMapServer.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_SERVER_KNOWNMAPSERVER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_SERVER_KNOWNMAPSERVER_H_

#include <boost/shared_ptr.hpp>

#include "preprocessing/KnownMapPreprocessor.h"

namespace known_map_localization {
namespace known_map_server {

class KnownMapServer {
public:
	KnownMapServer(preprocessing::KnownMapPreprocessorPtr preprocessor);
};

typedef boost::shared_ptr<KnownMapServer> KnownMapServerPtr;
typedef boost::shared_ptr<KnownMapServer const> KnownMapServerConstPtr;
} /* namespace known_map_server */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_SERVER_KNOWNMAPSERVER_H_ */
