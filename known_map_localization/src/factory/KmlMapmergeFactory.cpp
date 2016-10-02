/*
 * KmlMapmergeFactory.cpp
 *
 *  Created on: 02.10.2016
 *      Author: jacob
 */

#include <factory/KmlMapmergeFactory.h>

#include <aligning/MapmergeAligner.h>
#include <preprocessing/MapmergeSlamMapPreprocessor.h>
#include <filter/GpsFilter.h>
#include <preprocessing/EdgeDetectionKnownMapPreprocessor.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlMapmergeFactory::createAligner() const {
	return make_shared<MapmergeAligner>();
}

KnownMapPreprocessorPtr KmlMapmergeFactory::createKnownMapPreprocessor() const {
	return make_shared<EdgeDetectionKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlMapmergeFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager) const {
	return make_shared<MapmergeSlamMapPreprocessor>(pSlamScaleManager);
}

FilterPtr KmlMapmergeFactory::createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger) const {
	return make_shared<GpsFilter>(pGpsManager, pKnownMapServer, pSlamScaleManager, pDataLogger);
}

} /* namespace kml */
