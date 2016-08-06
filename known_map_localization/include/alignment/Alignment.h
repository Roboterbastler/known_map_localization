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

/**
 * # Alignment
 * Represents an alignment of two maps.
 */
class Alignment {
public:
	/**
	 * Converts the alignment to a tf transform.
	 * @note The scale factor is omitted when converting!
	 * @return The tf transform
	 */
	tf::Transform toTfTransform() const;

	/// The scale
	float scale;

	/// The rotation angle (yaw) in radians
	float theta;

	/// Translation x
	float x;

	/// Translation y
	float y;

	/// Origin map frame ID
	std::string from;

	// Target map frame ID
	std::string to;

	/**
	 * Get an alignment without any transformation.
	 * @return The identity alignment
	 */
	static Alignment getIdentity();
};

} /* namespace alignment */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_ALIGNMENT_H_ */
