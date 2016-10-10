/*
 * CsMergeIcpSvdAligner.h
 *
 *  Created on: 10.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEICPSVDALIGNER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEICPSVDALIGNER_H_

#include <aligning/CsMergeAligner.h>

namespace kml {

class CsMergeIcpSvdAligner: public CsMergeAligner {
public:
	CsMergeIcpSvdAligner();
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEICPSVDALIGNER_H_ */
