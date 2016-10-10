/*
 * KmlCsMergeHoughCcrFactory.cpp
 *
 *  Created on: 10.10.2016
 *      Author: jacob
 */

#include <factory/KmlCsMergeHoughCcrFactory.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlCsMergeHoughCcrFactory::createAligner() const {
	return make_shared<CsMergeHoughCcrAligner>();
}

KnownMapPreprocessorPtr KmlCsMergeHoughCcrFactory::createKnownMapPreprocessor() const {
	return make_shared<EdgeDetectionKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlCsMergeHoughCcrFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager) const {
	return make_shared<MapmergeSlamMapPreprocessor>(pSlamScaleManager);
}

FilterPtr KmlCsMergeHoughCcrFactory::createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger) const {
	return make_shared<GpsFilter>(pGpsManager, pKnownMapServer, pSlamScaleManager, pDataLogger);
}

} /* namespace kml */
