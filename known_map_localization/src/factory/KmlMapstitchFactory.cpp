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
#include <preprocessing/EdgeDetectionKnownMapPreprocessor.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlMapstitchFactory::createAligner() const {
	return make_shared<MapstitchAligner>();
}

KnownMapPreprocessorPtr KmlMapstitchFactory::createKnownMapPreprocessor() const {
	return make_shared<EdgeDetectionKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlMapstitchFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<MapstitchSlamMapPreprocessor>(pSlamScaleManager, pKnownMapServer);
}

} /* namespace kml */
