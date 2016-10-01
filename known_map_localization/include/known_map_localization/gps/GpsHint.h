/*
 * GpsHint.h
 *
 *  Created on: 01.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPS_GPSHINT_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPS_GPSHINT_H_

#include <gps/GpsPosition.h>

#include <tf/tf.h>

namespace kml {

struct GpsHint: GpsPosition {
	/// the associated SLAM base link (not scaled!)
	tf::Transform baseLink;
};

typedef std::vector<GpsHint> GpsHintVect;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPS_GPSHINT_H_ */
