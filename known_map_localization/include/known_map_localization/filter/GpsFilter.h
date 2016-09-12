/*
 * GpsFilter.h
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPose.h>

#include <filter/Filter.h>
#include <Exception.h>

namespace known_map_localization {
namespace filter {

/**
 * # GPS Filter
 *
 * ## Parameters
 * - **gps_constraint_radius**: The radius of the constraint given by a GPS fix
 *
 * ## Subscribed topics
 * - __/robot/gps__: The GPS signal
 */
class GpsFilter : public Filter {
public:
	GpsFilter();

	/**
	 * Update the filtered alignment by simply overwriting it with the first new hypothesis.
	 * @param alignment The new alignment
	 */
	void addHypotheses(const alignment::HypothesesVect &hypotheses);

	/**
	 * Converts a GPS fix to the known map anchor frame. If the GPS fix lies in a different UTM grid zone
	 * than the known map anchor an exception is thrown.
	 * @param gpsFix The GPS fix
	 * @param anchor The known map anchor GeoPose
	 * @return The position in the anchor frame
	 * @throws DifferentUTMGridZones
	 */
	static geometry_msgs::PointStamped convertGPSPositionToAnchorFrame(const sensor_msgs::NavSatFix &gpsFix, const geographic_msgs::GeoPose &anchor);

protected:
	void receiveGpsFix(const sensor_msgs::NavSatFix &gpsFix);

private:
	/// Subscribes to the GPS topic
	ros::Subscriber gpsSubscriber;

	/// The radius of the constraint given by a GPS fix
	float constraintRadius;
};

} /* namespace filter */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_ */
