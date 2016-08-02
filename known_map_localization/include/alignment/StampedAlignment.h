/*
 * StampedAlignment.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_STAMPEDALIGNMENT_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_STAMPEDALIGNMENT_H_

#include <ros/time.h>

#include "Alignment.h"

namespace known_map_localization {
namespace alignment {

class StampedAlignment: public Alignment {
public:
	tf::StampedTransform toTfStampedTransform() const;

	ros::Time stamp;
};

} /* namespace alignment */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALIGNMENT_STAMPEDALIGNMENT_H_ */
