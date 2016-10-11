/*
 * GpsFilter.h
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <filter/Filter.h>
#include <gps/GpsManager.h>
#include <alignment/GpsScoredHypothesis.h>

namespace kml {

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
	GpsFilter(GpsManagerConstPtr pGpsManager,
			KnownMapServerConstPtr pKnownMapServer,
			SlamScaleManagerPtr pSlamScaleManager, DataLoggerPtr pDataLogger =
					DataLoggerPtr());

	/**
	 * Update the filtered alignment by simply overwriting it with the first new hypothesis.
	 * @param alignment The new alignment
	 */
	void addHypotheses(const HypothesesVect &hypotheses);

	/**
	 * Get the filtered alignment, if available. If it isn't available, an exception is thrown.
	 * @return The filtered alignment
	 * @throw AlignmentNotAvailable, if filtered alignment is not available
	 */
	const Alignment& getAlignment() const;

private:
	/**
	 * Computes the estimated robot pose in the anchor frame, based on the
	 * given map alignment, GPS hint and the scaled SLAM base link.
	 * @param alignment The map alignment to be used
	 * @param hint The GPS hint to be used
	 * @return The robot's estimated pose
	 */
	geometry_msgs::Pose estimatedRobotPose(const Alignment &alignment, const GpsHint &hint) const;

	/**
	 * Computes a scoring for a given hypothesis.
	 * @param h The hypothesis
	 * @param constraints The marker message to add visualization info to
	 */
	void scoringFunction(GpsScoredHypothesis &h, visualization_msgs::Marker &constraints) const;

	/**
	 * Sets up a marker for the GPS constraints.
	 * @return The marker
	 */
	visualization_msgs::Marker setUpContraintMarker() const;

	/**
	 * Adds a marker for a GPS hint constraint.
	 * @param hint The GPS constraint
	 * @param marker The marker
	 * @param supporting True, if constraint supports alignment, false if it impairs it
	 */
	void addConstraintMarker(const GpsHint &hint, visualization_msgs::Marker &marker, bool supporting) const;

private:
	/// The last accepted scored hypothesis, used instead of the filteredAlignment of the base class Filter
	GpsScoredHypothesis mFilteredGpsHypothesis_;

	/// Visualization marker for GPS constraints
	visualization_msgs::Marker mConstraintsMarker_;

	/// Publishes marker for visualization/debugging purposes
	ros::Publisher mGpsConstraintsMarkerPublisher_;

	/// The radius of the constraint given by a GPS fix
	float kGpsConstraintRadius_;

	float kDecayFactor_;

	float kConfirmationFactor_;

	bool kPreferGpsSupported_;

private:
	GpsManagerConstPtr pGpsManager_;
	KnownMapServerConstPtr pKnownMapServer_;
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_FILTER_GPSFILTER_H_ */
