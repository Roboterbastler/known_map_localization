/*
 * KnownMapLocalization.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include "KnownMapLocalization.h"
#include <filter/PassThroughFilter.h>

namespace known_map_localization {
using namespace known_map_server;

KnownMapLocalization::KnownMapLocalization() :
		algorithmSelector(new AlgorithmSelector()),
		knownMapServer(new KnownMapServer(algorithmSelector->getKnownMapPreprocessor())),
		filter(new filter::PassThroughFilter()),
		baseLinkPublisher(new base_link::BaseLinkPublisher(filter, ros::WallDuration(0.5))) {
}

} /* namespace known_map_localization */
