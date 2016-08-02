/*
 * KnownMapLocalization.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include "KnownMapLocalization.h"

namespace known_map_localization {
using namespace known_map_server;

KnownMapLocalization::KnownMapLocalization() :
		knownMapServer(new KnownMapServer(algorithmSelector->getKnownMapPreprocessor())),
		baseLinkPublisher(filter, ros::Duration(0.5)) {
}

} /* namespace known_map_localization */
