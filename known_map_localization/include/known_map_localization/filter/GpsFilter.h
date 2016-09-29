/*
 * GpsFilter.h
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_

#include <alignment/GpsScoredHypothesis.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <filter/Filter.h>
#include <GpsManager.h>

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
	 * Get the filtered alignment, if available. If it isn't available, an exception is thrown.
	 * @return The filtered alignment
	 * @throw AlignmentNotAvailable, if filtered alignment is not available
	 */
	const alignment::Alignment& getAlignment() const;

protected:

	/**
	 * Computes a scoring for a given hypothesis.
	 * @param h The hypothesis
	 * @param constraints The marker message to add visualization info to
	 */
	void scoringFunction(alignment::GpsScoredHypothesis &h, visualization_msgs::Marker &constraints) const;

	/**
	 * Sets up a marker for the GPS constraints.
	 * @return The marker
	 */
	visualization_msgs::Marker setUpContraintMarker() const;

	/**
	 * Adds a marker for a GPS key point constraint.
	 * @param hint The GPS constraint
	 * @param marker The marker
	 * @param supporting True, if constraint supports alignment, false if it impairs it
	 */
	void addConstraintMarker(const GpsKeyPoint &hint, visualization_msgs::Marker &marker, bool supporting) const;

private:
	/// The last accepted scored hypothesis, used instead of the filteredAlignment of the base class Filter
	alignment::GpsScoredHypothesis filteredHypothesis;

	/// Visualization marker for GPS constraints
	visualization_msgs::Marker constraintsMarker;

	/// Publishes marker for visualization/debugging purposes
	ros::Publisher gpsConstraintsMarkerPublisher;

	/// The radius of the constraint given by a GPS fix
	float CONSTRAINT_RADIUS;

	float AGING_RATE;

	float CONFIRMATION_FACTOR;
};

} /* namespace filter */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_ */
