/*
 * KmlFactory.h
 *
 *  Created on: 01.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLFACTORY_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLFACTORY_H_

#include <boost/shared_ptr.hpp>

#include <aligning/Aligner.h>
#include <preprocessing/KnownMapPreprocessor.h>
#include <preprocessing/SlamMapPreprocessor.h>
#include <filter/Filter.h>
#include <localization/Localization.h>
#include <localization/PoseErrorPublisher.h>
#include <known_map_server/KnownMapServer.h>
#include <logging/DataLogger.h>
#include <gps/GpsManager.h>
#include <SlamScaleManager.h>
#include <StatusPublisher.h>

namespace kml {

class KmlFactory {
public:
	virtual ~KmlFactory();

	virtual AlignerPtr createAligner() const = 0;
	virtual KnownMapPreprocessorPtr createKnownMapPreprocessor() const = 0;
	virtual SlamMapPreprocessorPtr createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const = 0;
	virtual FilterPtr createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, StatusPublisherPtr pStatusPublisher, DataLoggerPtr pDataLogger = DataLoggerPtr()) const;
	virtual LocalizationPtr createLocalization(FilterConstPtr pFilter, SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const;
	virtual PoseErrorPublisherPtr createPoseErrorPublisher(DataLoggerPtr pDataLogger) const;
	virtual KnownMapServerPtr createKnownMapServer(KnownMapPreprocessorPtr pKnownMapPreprocessor) const;
	virtual DataLoggerPtr createDataLogger() const;
	virtual GpsManagerPtr createGpsManager(KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager) const;
	virtual SlamScaleManagerPtr createSlamScaleManager(GpsManagerConstPtr pGpsManager, DataLoggerPtr pDataLogger = DataLoggerPtr()) const;
	virtual StatusPublisherPtr createStatusPublisher(DataLoggerPtr pDataLogger, float rate = 10) const;
};

typedef boost::shared_ptr<KmlFactory const> KmlFactoryConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FACTORY_KMLFACTORY_H_ */
