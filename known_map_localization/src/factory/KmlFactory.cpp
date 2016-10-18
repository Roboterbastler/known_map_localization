/*
 * KmlFactory.cpp
 *
 *  Created on: 01.10.2016
 *      Author: jacob
 */

#include <factory/KmlFactory.h>

#include <filter/GpsFilter.h>

namespace kml {

using boost::make_shared;

KmlFactory::~KmlFactory() {

}

FilterPtr KmlFactory::createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, StatusPublisherPtr pStatusPublisher, DataLoggerPtr pDataLogger) const {
	return make_shared<GpsFilter>(pGpsManager, pKnownMapServer, pSlamScaleManager, pStatusPublisher, pDataLogger);
}

BaseLinkPublisherPtr KmlFactory::createBaseLinkPublisher(KnownMapServerConstPtr pKnownMapServer, FilterConstPtr pFilter, SlamScaleManagerConstPtr pSlamScaleManager, DataLoggerPtr pDataLogger) const {
	return make_shared<BaseLinkPublisher>(pKnownMapServer, pFilter, pSlamScaleManager, pDataLogger);
}

KnownMapServerPtr KmlFactory::createKnownMapServer(KnownMapPreprocessorPtr pKnownMapPreprocessor) const {
	return make_shared<KnownMapServer>(pKnownMapPreprocessor);
}

DataLoggerPtr KmlFactory::createDataLogger() const {
	return make_shared<DataLogger>();
}

GpsManagerPtr KmlFactory::createGpsManager(KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<GpsManager>(pKnownMapServer);
}

SlamScaleManagerPtr KmlFactory::createSlamScaleManager(GpsManagerConstPtr pGpsManager, DataLoggerPtr pDataLogger) const {
	return make_shared<SlamScaleManager>(pGpsManager, pDataLogger);
}

StatusPublisherPtr KmlFactory::createStatusPublisher(float rate) const {
	return make_shared<StatusPublisher>(rate);
}

} /* namespace kml */
