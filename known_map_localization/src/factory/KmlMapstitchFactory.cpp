/*
 * KmlMapstitchFactory.cpp
 *
 *  Created on: 16.10.2016
 *      Author: jacob
 */

#include <factory/KmlMapstitchFactory.h>

#include <aligning/MapstitchAligner.h>
#include <preprocessing/MapstitchSlamMapPreprocessor.h>
#include <filter/GpsFilter.h>
#include <preprocessing/MapstitchKnownMapPreprocessor.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlMapstitchFactory::createAligner() const {
	return make_shared<MapstitchAligner>();
}

KnownMapPreprocessorPtr KmlMapstitchFactory::createKnownMapPreprocessor() const {
	return make_shared<MapstitchKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlMapstitchFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<MapstitchSlamMapPreprocessor>(pSlamScaleManager, pKnownMapServer);
}

FilterPtr KmlMapstitchFactory::createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger) const {
	return make_shared<GpsFilter>(pGpsManager, pKnownMapServer, pSlamScaleManager, pDataLogger);
}

} /* namespace kml */
