/*
 * KmlCsMergeHoughCorner.cpp
 *
 *  Created on: 03.10.2016
 *      Author: jacob
 */

#include <aligning/CsMergeHoughCornerAligner.h>
#include <factory/KmlCsMergeHoughCornerFactory.h>
#include <preprocessing/EdgeDetectionKnownMapPreprocessor.h>
#include <preprocessing/MapmergeSlamMapPreprocessor.h>
#include <filter/GpsFilter.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlCsMergeHoughCornerFactory::createAligner() const {
	return make_shared<CsMergeHoughCornerAligner>();
}

KnownMapPreprocessorPtr KmlCsMergeHoughCornerFactory::createKnownMapPreprocessor() const {
	return make_shared<EdgeDetectionKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlCsMergeHoughCornerFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager) const {
	return make_shared<MapmergeSlamMapPreprocessor>(pSlamScaleManager);
}

FilterPtr KmlCsMergeHoughCornerFactory::createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger) const {
	return make_shared<GpsFilter>(pGpsManager, pKnownMapServer, pSlamScaleManager, pDataLogger);
}

} /* namespace kml */
