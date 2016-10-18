/*
 * KmlCsMergeIcpSvdFactory.h
 *
 *  Created on: 10.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLCSMERGEICPSVDFACTORY_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLCSMERGEICPSVDFACTORY_H_

#include <factory/KmlCsMergeFactory.h>

namespace kml {

class KmlCsMergeIcpSvdFactory: public KmlCsMergeFactory {
public:
	AlignerPtr createAligner() const;
	KnownMapPreprocessorPtr createKnownMapPreprocessor() const;
	SlamMapPreprocessorPtr createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const;
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLCSMERGEICPSVDFACTORY_H_ */
