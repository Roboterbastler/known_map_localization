/*
 * KmlCsMergeIcpSvdFactory.cpp
 *
 *  Created on: 10.10.2016
 *      Author: jacob
 */

#include <factory/KmlCsMergeIcpSvdFactory.h>

#include <aligning/CsMergeIcpSvdAligner.h>
#include <preprocessing/EdgeDetectionKnownMapPreprocessor.h>
#include <preprocessing/MapmergeSlamMapPreprocessor.h>
#include <filter/GpsFilter.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlCsMergeIcpSvdFactory::createAligner() const {
	return make_shared<CsMergeIcpSvdAligner>();
}

KnownMapPreprocessorPtr KmlCsMergeIcpSvdFactory::createKnownMapPreprocessor() const {
	return make_shared<EdgeDetectionKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlCsMergeIcpSvdFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<MapmergeSlamMapPreprocessor>(pSlamScaleManager, pKnownMapServer);
}

FilterPtr KmlCsMergeIcpSvdFactory::createFilter(GpsManagerConstPtr pGpsManager, KnownMapServerConstPtr pKnownMapServer, SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger) const {
	return make_shared<GpsFilter>(pGpsManager, pKnownMapServer, pSlamScaleManager, pDataLogger);
}

} /* namespace kml */
