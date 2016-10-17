/*
 * KmlFactory.cpp
 *
 *  Created on: 01.10.2016
 *      Author: jacob
 */

#include <factory/KmlFactory.h>

namespace kml {

KmlFactory::~KmlFactory() {

}

BaseLinkPublisherPtr KmlFactory::createBaseLinkPublisher(KnownMapServerConstPtr pKnownMapServer, FilterConstPtr pFilter, SlamScaleManagerConstPtr pSlamScaleManager, DataLoggerPtr pDataLogger) const {
	return BaseLinkPublisherPtr(new BaseLinkPublisher(pKnownMapServer, pFilter, pSlamScaleManager, pDataLogger));
}

KnownMapServerPtr KmlFactory::createKnownMapServer(KnownMapPreprocessorPtr pKnownMapPreprocessor) const {
	return KnownMapServerPtr(new KnownMapServer(pKnownMapPreprocessor));
}

DataLoggerPtr KmlFactory::createDataLogger() const {
	return DataLoggerPtr(new DataLogger());
}

GpsManagerPtr KmlFactory::createGpsManager(KnownMapServerConstPtr pKnownMapServer) const {
	return GpsManagerPtr(new GpsManager(pKnownMapServer));
}

SlamScaleManagerPtr KmlFactory::createSlamScaleManager(GpsManagerConstPtr pGpsManager, DataLoggerPtr pDataLogger) const {
	return SlamScaleManagerPtr(new SlamScaleManager(pGpsManager, pDataLogger));
}

StatusPublisherPtr KmlFactory::createStatusPublisher(float rate) const {
	return boost::make_shared<StatusPublisher>(rate);
}

} /* namespace kml */
