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

SlamMapPreprocessorPtr KmlCsMergeHoughCornerFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<MapmergeSlamMapPreprocessor>(pSlamScaleManager, pKnownMapServer);
}

} /* namespace kml */
