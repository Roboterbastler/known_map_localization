/*
 * KnownMapLocalization.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_

#include <AlgorithmSelector.h>
#include <known_map_server/KnownMapServer.h>
#include <filter/Filter.h>
#include <base_link/BaseLinkPublisher.h>

namespace known_map_localization {

class KnownMapLocalization {
public:
	KnownMapLocalization();
private:
	/// selects the algorithms according to the method chosen
	AlgorithmSelectorPtr algorithmSelector;

	/// publishes the known map
	known_map_server::KnownMapServerConstPtr knownMapServer;

	/// filters the computed alignments
	filter::FilterPtr filter;

	/// publishes the base link based on the filtered alignment
	base_link::BaseLinkPublisherConstPtr baseLinkPublisher;
};

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWNMAPLOCALIZATION_H_ */
