/*
 * KmlCsMergeIcpGradientFactory.cpp
 *
 *  Created on: 02.10.2016
 *      Author: jacob
 */

#include <factory/KmlCsMergeIcpGradientFactory.h>

#include <aligning/CsMergeIcpGradientAligner.h>
#include <preprocessing/EdgeDetectionKnownMapPreprocessor.h>
#include <preprocessing/MapmergeSlamMapPreprocessor.h>
#include <filter/GpsFilter.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlCsMergeIcpGradientFactory::createAligner() const {
	return make_shared<CsMergeIcpGradientAligner>();
}

KnownMapPreprocessorPtr KmlCsMergeIcpGradientFactory::createKnownMapPreprocessor() const {
	return make_shared<EdgeDetectionKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlCsMergeIcpGradientFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<MapmergeSlamMapPreprocessor>(pSlamScaleManager, pKnownMapServer);
}

FilterPtr KmlCsMergeIcpGradientFactory::createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger) const {
	return make_shared<GpsFilter>(pGpsManager, pKnownMapServer, pSlamScaleManager, pDataLogger);
}

} /* namespace kml */
