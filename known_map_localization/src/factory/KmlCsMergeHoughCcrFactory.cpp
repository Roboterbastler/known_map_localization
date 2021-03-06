/*
 * KmlCsMergeHoughCcrFactory.cpp
 *
 *  Created on: 10.10.2016
 *      Author: jacob
 */

#include <factory/KmlCsMergeHoughCcrFactory.h>

#include <aligning/CsMergeHoughCcrAligner.h>
#include <preprocessing/EdgeDetectionKnownMapPreprocessor.h>
#include <preprocessing/MapmergeSlamMapPreprocessor.h>
#include <filter/GpsFilter.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlCsMergeHoughCcrFactory::createAligner() const {
	return make_shared<CsMergeHoughCcrAligner>();
}

KnownMapPreprocessorPtr KmlCsMergeHoughCcrFactory::createKnownMapPreprocessor() const {
	return make_shared<EdgeDetectionKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlCsMergeHoughCcrFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<MapmergeSlamMapPreprocessor>(pSlamScaleManager, pKnownMapServer);
}

} /* namespace kml */
