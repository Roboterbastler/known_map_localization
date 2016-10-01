/*
 * CsMergeIcpGradient.h
 *
 *  Created on: 30.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEICPGRADIENT_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEICPGRADIENT_H_

#include <aligning/CsMergeAligner.h>

namespace known_map_localization {
namespace aligning {

class CsMergeIcpGradient: public CsMergeAligner {
public:
	CsMergeIcpGradient();
};

} /* namespace aligning */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_ALIGNING_CSMERGEICPGRADIENT_H_ */
