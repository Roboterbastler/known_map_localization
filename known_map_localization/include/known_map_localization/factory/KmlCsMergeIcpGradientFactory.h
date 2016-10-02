/*
 * KmlCsMergeIcpGradientFactory.h
 *
 *  Created on: 02.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLCSMERGEICPGRADIENTFACTORY_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLCSMERGEICPGRADIENTFACTORY_H_

#include <factory/KmlCsMergeFactory.h>

namespace kml {

class KmlCsMergeIcpGradientFactory: public KmlCsMergeFactory {
public:
	AlignerPtr createAligner() const;
	KnownMapPreprocessorPtr createKnownMapPreprocessor() const;
	SlamMapPreprocessorPtr createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager) const;
	FilterPtr createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger = DataLoggerPtr()) const;
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLCSMERGEICPGRADIENTFACTORY_H_ */
