/*
 * StampedAlignment.cpp
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#include "StampedAlignment.h"

namespace known_map_localization {
namespace alignment {

tf::StampedTransform StampedAlignment::toTfStampedTransform() const {
	return tf::StampedTransform(toTfTransform(), stamp, from, to);
}

} /* namespace alignment */
} /* namespace known_map_localization */
