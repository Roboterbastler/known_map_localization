/*
 * KmlCsMergeIcpGradientFactory.cpp
 *
 *  Created on: 02.10.2016
 *      Author: jacob
 */

#include <factory/KmlCsMergeIcpGradientFactory.h>

#include <aligning/CsMergeIcpGradientAligner.h>
#include <preprocessing/IcpKnownMapPreprocessor.h>
#include <preprocessing/IcpSlamMapPreprocessor.h>
#include <filter/GpsFilter.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlCsMergeIcpGradientFactory::createAligner() const {
	return make_shared<CsMergeIcpGradientAligner>();
}

KnownMapPreprocessorPtr KmlCsMergeIcpGradientFactory::createKnownMapPreprocessor() const {
	return make_shared<IcpKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlCsMergeIcpGradientFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<IcpSlamMapPreprocessor>(pSlamScaleManager, pKnownMapServer);
}

} /* namespace kml */
