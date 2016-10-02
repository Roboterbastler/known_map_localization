/*
 * KmlFactory.h
 *
 *  Created on: 01.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLFACTORY_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLFACTORY_H_

#include <aligning/Aligner.h>
#include <preprocessing/KnownMapPreprocessor.h>
#include <preprocessing/SlamMapPreprocessor.h>
#include <filter/Filter.h>
#include <base_link/BaseLinkPublisher.h>
#include <known_map_server/KnownMapServer.h>
#include <logging/DataLogger.h>
#include <gps/GpsManager.h>
#include <SlamScaleManager.h>

namespace kml {

class KmlFactory {
public:
	virtual ~KmlFactory();

	virtual AlignerPtr createAligner() const = 0;
	virtual KnownMapPreprocessorPtr createKnownMapPreprocessor() const = 0;
	virtual SlamMapPreprocessorPtr createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager) const = 0;
	virtual FilterPtr createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger = DataLoggerPtr()) const = 0;
	virtual BaseLinkPublisherPtr createBaseLinkPublisher(KnownMapServerConstPtr pKnownMapServer, FilterConstPtr pFilter, SlamScaleManagerConstPtr pSlamScaleManager, DataLoggerPtr pDataLogger = DataLoggerPtr()) const;
	virtual KnownMapServerPtr createKnownMapServer(KnownMapPreprocessorPtr pKnownMapPreprocessor) const;
	virtual DataLoggerPtr createDataLogger() const;
	virtual GpsManagerPtr createGpsManager(KnownMapServerConstPtr pKnownMapServer) const;
	virtual SlamScaleManagerPtr createSlamScaleManager(GpsManagerConstPtr pGpsManager, DataLoggerPtr pDataLogger = DataLoggerPtr()) const;
};

typedef boost::shared_ptr<KmlFactory const> KmlFactoryConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLFACTORY_H_ */