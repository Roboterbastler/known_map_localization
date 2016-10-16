/*
 * KmlMapmergeFactory.h
 *
 *  Created on: 02.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLMAPMERGEFACTORY_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLMAPMERGEFACTORY_H_

#include <factory/KmlFactory.h>

namespace kml {

/**
 * # Mapmerge factory
 *
 * Creates a setup with the mapmerge algorithm and the GPS filter.
 */
class KmlMapmergeFactory: public KmlFactory {
public:
	AlignerPtr createAligner() const;
	KnownMapPreprocessorPtr createKnownMapPreprocessor() const;
	SlamMapPreprocessorPtr createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const;
	FilterPtr createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger = DataLoggerPtr()) const;
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLMAPMERGEFACTORY_H_ */
