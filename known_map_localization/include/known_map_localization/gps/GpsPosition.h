/*
 * GpsPosition.h
 *
 *  Created on: 01.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPS_GPSPOSITION_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPS_GPSPOSITION_H_

#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>

namespace kml {

struct GpsPosition {
	/// the time stamp of the associated positions
	ros::Time stamp;

	/// the GPS fix
	sensor_msgs::NavSatFix gpsFix;

	/// GPS position in the anchor frame
	geometry_msgs::Point gpsPosition;
};

typedef std::vector<GpsPosition> GpsPositionVect;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPS_GPSPOSITION_H_ */
