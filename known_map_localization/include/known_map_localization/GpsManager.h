/*
 * GpsManager.h
 *
 *  Created on: 16.09.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPSMANAGER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPSMANAGER_H_

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPose.h>
#include <tf/transform_listener.h>

#include <alignment/Alignment.h>

namespace known_map_localization {

class GpsKeyPoint {
public:
	/**
	 * Computes the estimated robot pose in the anchor frame, based on the
	 * given map alignment and the scaled SLAM base link.
	 * @param alignment The map alignment to be used
	 * @return The robot's estimated pose
	 */
	geometry_msgs::Pose getRobotPose(const alignment::Alignment &alignment) const;

	/// the GPS fix
	sensor_msgs::NavSatFix gpsFix;

	/// GPS position in the anchor frame
	geometry_msgs::Point gpsPosition;

	/// the associated SLAM base link (not scaled!)
	tf::Transform baseLink;

	/// the time stamp of the associated positions
	ros::Time stamp;
};

typedef std::vector<GpsKeyPoint> GpsKeyPointVect;

class GpsManager;
typedef boost::shared_ptr<GpsManager> GpsManagerPtr;
typedef boost::shared_ptr<GpsManager const> GpsManagerConstPtr;

/**
 * # GPS Manager
 *
 * ## Subscribed topics
 * - __/gps_fix__: The GPS fix topic
 */
class GpsManager {
public:
	static GpsManagerPtr instance();

	/**
	 * Gives read-only access to the stored GPS position/SLAM base link key points.
	 * @return Vector
	 */
	const GpsKeyPointVect& getKeyPoints() const;

protected:
	GpsManager();

private:
	/**
	 * Callback function to receive GPS fixes
	 * @param fix The GPS fix message
	 */
	void receiveGpsFix(const sensor_msgs::NavSatFix &fix);

	/**
	 * Timer callback function.
	 * @param event
	 */
	void updateKeyPoints(const ros::WallTimerEvent& event);

	/**
	 * Returns the SLAM base link for the requested time.
	 * @param t The requested time
	 * @return The transformation
	 */
	tf::StampedTransform getSlamBaseLink(ros::Time t);

	/**
	 * Converts a GPS fix to the known map anchor frame. If the GPS fix lies in a different UTM grid zone
	 * than the known map anchor an exception is thrown.
	 * @param gpsFix The GPS fix
	 * @param anchor The known map anchor GeoPose
	 * @return The position in the anchor frame
	 * @throws DifferentUTMGridZones
	 */
	static geometry_msgs::Point convertGPSPositionToAnchorFrame(const sensor_msgs::NavSatFix &gpsFix, const geographic_msgs::GeoPose &anchor);

private:
	static GpsManagerPtr _instance;

	/// Transform listener
	tf::TransformListener listener;

	ros::WallTimer timer;

	/// Subscriber for the GPS fix topic
	ros::Subscriber gpsFixSubscriber;

	/// queue of GPS fixes waiting to be paired with the according SLAM base links
	std::vector<sensor_msgs::NavSatFix> fixQueue;

	/// GPS position/SLAM base link pairs
	std::vector<GpsKeyPoint> keypoints;
};

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPSMANAGER_H_ */
