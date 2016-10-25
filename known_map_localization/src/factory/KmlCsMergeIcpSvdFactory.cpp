/*
 * KmlCsMergeIcpSvdFactory.cpp
 *
 *  Created on: 10.10.2016
 *      Author: jacob
 */

#include <factory/KmlCsMergeIcpSvdFactory.h>

#include <aligning/CsMergeIcpSvdAligner.h>
#include <preprocessing/IcpKnownMapPreprocessor.h>
#include <preprocessing/IcpSlamMapPreprocessor.h>
#include <filter/GpsFilter.h>

namespace kml {

using boost::make_shared;

AlignerPtr KmlCsMergeIcpSvdFactory::createAligner() const {
	return make_shared<CsMergeIcpSvdAligner>();
}

KnownMapPreprocessorPtr KmlCsMergeIcpSvdFactory::createKnownMapPreprocessor() const {
	return make_shared<IcpKnownMapPreprocessor>();
}

SlamMapPreprocessorPtr KmlCsMergeIcpSvdFactory::createSlamMapPreprocessor(SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer) const {
	return make_shared<IcpSlamMapPreprocessor>(pSlamScaleManager, pKnownMapServer);
}

} /* namespace kml */
