/*
 * SlamScaleManager.h
 *
 *  Created on: 27.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_SLAMSCALEMANAGER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_SLAMSCALEMANAGER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <gps/GpsManager.h>
#include <logging/DataLogger.h>

namespace kml {

/// The mode of the SLAM scale manager
typedef enum {
	INVALID = 0, ///< Invalid mode only for internal use
	PARAMETER, ///< Uses the **slam_map_scale** parameter as a fixed SLAM scale
	ALIGNMENT, ///< Uses the scale estimates provided by the aligning algorithms (not all algorithms support that)
	GPS ///< Uses GPS fixes to estimate the SLAM scale
} SlamScaleMode;

/**
 * # SLAM Scale Manager
 *
 * This class manages the SLAM scale, that can come from different sources.
 *
 * It implements the Singleton design pattern.
 *
 * ## Parameters
 * - **slam_scale_mode**: The mode of the SLAM scale manager
 * - **slam_map_scale**: The a priori known scale of the SLAM map, e.g. used with the mapmerge algorithm.
 * Only used in PARAMETER mode.
 *
 * ## Subscribed Topics
 * - __/robot/gps__ (only in GPS mode): GPS fixes are received via this topic
 * - **gps_hints_updated**: Signal that the GPS key points have been updated
 */
class SlamScaleManager {
public:
	SlamScaleManager(GpsManagerConstPtr pGpsManager, DataLoggerPtr pDataLogger = DataLoggerPtr());

	/**
	 * Gets the SLAM scale
	 * @return SLAM scale
	 * @throws ScaleNotAvailable if no scale (estimate) is available.
	 */
	float getSlamScale() const;

	/**
	 * Gets the current mode.
	 * @return The mode
	 */
	SlamScaleMode getMode() const;

	/**
	 * Updates the SLAM scale determined by the aligning process.
	 * @param scale The new SLAM scale estimated by the aligning algorithm
	 * @note This method has no effect if the SLAM scale manager is not in the ALIGNMENT mode.
	 */
	void updateSlamScale(float scale);

	/**
	 * Converts a transformation from ORB SLAM's coordinate system to real world coordinates.
	 * @param transform The transformation to convert
	 * @return The converted transformation
	 */
	tf::Transform convertTransform(tf::Transform transform) const;

	/**
	 * Converts a transformation from ORB SLAM's coordinate system to real world coordinates.
	 * @param transform The transformation to convert
	 * @return The converted transformation
	 */
	tf::StampedTransform convertTransform(tf::StampedTransform transform) const;

	/**
	 * Converts a pose from ORB SLAM's coordinate system to real world coordinates.
	 * @param pose The pose to be converted
	 * @return The resulting pose
	 */
	geometry_msgs::Pose convertPoseMsg(geometry_msgs::Pose pose) const;

protected:
	/**
	 * Tries to read the mode parameter and returns the according mode.
	 * @return The mode
	 */
	SlamScaleMode determineMode() const;

	/**
	 * Is called when a message arrives that signals that the GPS hints have been updated.
	 *
	 * Computes an estimate of the SLAM map scale by comparing two distances.
	 * The distance between two GPS fixes and the distance between the corresponding
	 * SLAM map positions. The position pair with the greatest distance is chosen in
	 * order to minimize influence of position inaccuracy.
	 *
	 * If a scale estimate was computed, the scale is updated.
	 * @param signal Empty message working as a signal
	 */
	void estimateScale(const std_msgs::Empty &signal);

private:
	/// The mode of the SLAM scale manager
	SlamScaleMode mMode_;

	/// This flag indicates if a scale (estimation) is available
	bool mIsValid_;

	/// The current scale estimation
	float mScale_;

	/// Listens to the **tf** topic
	tf::TransformListener mListener_;

	/// Signals that the GPS hints have been updated
	ros::Subscriber mGpsHintsUpdatedSubscriber_;

private:
	DataLoggerPtr pDataLogger_;
	GpsManagerConstPtr pGpsManager_;
};

typedef boost::shared_ptr<SlamScaleManager> SlamScaleManagerPtr;
typedef boost::shared_ptr<SlamScaleManager const> SlamScaleManagerConstPtr;

/**
 * Computes the distance between two points (on the xy plane).
 * @param p1 The first point
 * @param p2 The second point
 * @return The distance
 */
double distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

/**
 * Computes the median of a vector of values.
 * @param values The values
 * @return The median
 */
double median(std::vector<double> &values);

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_SLAMSCALEMANAGER_H_ */
