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
	GpsFilter(float constraintRadius, float maxAge, float agingFactor);

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

	/**
	 * Determines whether a hint is too old to be used as a constraint.
	 * @param hint The hint
	 * @return True if it is too old
	 */
	bool hintOutdated(const geometry_msgs::PointStamped &hint);

protected:
	/**
	 * Callback function for receiving GPS fixes
	 * @param gpsFix GPS fix message
	 */
	void receiveGpsFix(const sensor_msgs::NavSatFix &gpsFix);

	/**
	 * Computes a scoring for a given hypothesis.
	 * @param h The hypothesis
	 * @return The score
	 */
	float scoringFunction(const alignment::Hypothesis &h) const;

private:
	/// The score of the filtered alignment
	float filteredAlignmentScore;

	/// Subscribes to the GPS topic
	ros::Subscriber gpsSubscriber;

	/// The radius of the constraint given by a GPS fix
	float CONSTRAINT_RADIUS;

	float MAX_HINT_AGE;

	float AGING_RATE;

	float CONFIRMATION_FACTOR;

	/// Stores GPS fixes as hints for the filtering of the hypothesis
	std::vector<geometry_msgs::PointStamped> gpsPositionHints;
};

} /* namespace filter */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_ */
