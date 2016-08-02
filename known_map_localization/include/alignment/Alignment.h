/*
 * Alignment.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_ALIGNMENT_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_ALIGNMENT_H_

#include <string>

#include <tf/transform_datatypes.h>

namespace known_map_localization {
namespace alignment {

class Alignment {
public:
	tf::Transform toTfTransform() const;

	float scale;
	float theta;
	float x;
	float y;

	std::string from;
	std::string to;
};

} /* namespace alignment */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_ALIGNMENT_H_ */
