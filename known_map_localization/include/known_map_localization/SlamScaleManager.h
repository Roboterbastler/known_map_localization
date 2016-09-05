/*
 * SlamScaleManager.h
 *
 *  Created on: 27.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_SLAMSCALEMANAGER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_SLAMSCALEMANAGER_H_

#include <utility>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <geodesy/utm.h>

namespace known_map_localization {

namespace slam_scale_manager {

/// The mode of the SLAM scale manager
typedef enum {
	INVALID = 0, ///< Invalid mode only for internal use
	PARAMETER, ///< Uses the **slam_map_scale** parameter as a fixed SLAM scale
	ALIGNMENT, ///< Uses the scale estimates provided by the aligning algorithms (not all algorithms support that)
	GPS ///< Uses GPS fixes to estimate the SLAM scale
} SlamScaleMode;

typedef std::pair<geodesy::UTMPoint, geometry_msgs::Point> PositionPair;

class SlamScaleManager;
typedef boost::shared_ptr<SlamScaleManager> SlamScaleManagerPtr;
typedef boost::shared_ptr<SlamScaleManager const> SlamScaleManagerConstPtr;

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
 * - __/fix__ (only in GPS mode): GPS fixes are received via this topic
 */
class SlamScaleManager {
public:
	static SlamScaleManagerPtr instance();

	/**
	 * Gets the SLAM scale
	 * @return SLAM scale
	 * @throws ScaleNotAvailable if no scale (estimate) is available.
	 */
	float getSlamScale() const;

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

	/**
	 * Filter the position vectors and remove all positions where the GPS point is in a different
	 * UTM zone than the given GPS point.
	 * @param gpsPointData Vector of GPS positions
	 * @param slamMapPointData Vector of SLAM positions
	 * @param gpsPoint The given GPS point
	 */
	static void filterPositionData(std::vector<geodesy::UTMPoint> &gpsPointData, std::vector<geometry_msgs::Point> &slamMapPointData, const geodesy::UTMPoint &gpsPoint);

protected:
	SlamScaleManager();

	/**
	 * Callback method to receive GPS fixes.
	 * @param fix The GPS fix message
	 * @note This has no effect if the SLAM scale manager
	 * is not in the GPS mode.
	 */
	void receiveGpsFix(const sensor_msgs::NavSatFix &fix);

	/**
	 * Tries to read the mode parameter and returns the according mode.
	 * @return The mode
	 */
	SlamScaleMode determineMode() const;

	/**
	 * Performs least squares on the position data. Updates scale if least squares can be applied.
	 * @param pointData Positions from the GPS fixes and from SLAM base link
	 */
	void leastSquares(std::vector<PositionPair> pointData);

private:
	static SlamScaleManagerPtr _instance;

	/// The mode of the SLAM scale manager
	SlamScaleMode mode;

	/// This flag indicates if a scale (estimation) is available
	bool isValid;

	/// The current scale estimation
	float scale;

	/// The GPS fix subscriber
	ros::Subscriber gpsFixSubscriber;

	/// Listens to the **tf** topic
	tf::TransformListener listener;

	std::vector<PositionPair> pointData;
};

/**
 * Used to remove positions with a different UTM zone from a vector.
 */
class PositionUTMZoneFilter {
public:
	PositionUTMZoneFilter(uint8_t zone, char band) :
		zone(zone), band(band) {}

	bool operator() (const PositionPair &elem) const {
		return zone != elem.first.zone || band != elem.first.band;
	}
private:
	uint8_t zone;
	char band;
};

}

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_SLAMSCALEMANAGER_H_ */