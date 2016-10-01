/*
 * CsMergeAligner.h
 *
 *  Created on: 30.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_

#include <aligning/Aligner.h>

namespace kml {

/**
 * # cs_merge Aligner
 * This aligning algorithm uses the cs_merge package to align two maps.
 */
class CsMergeAligner: public Aligner {
protected:
	/// Service client for the aligning service call
	ros::ServiceClient mAligningClient_;
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEALIGNER_H_ */
