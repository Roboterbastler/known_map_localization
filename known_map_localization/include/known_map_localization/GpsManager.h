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

class GpsHint {
public:
	/**
	 * Computes the distance of the GPS hint position to another point.
	 * Both positions are assumed to be relative to the anchor frame.
	 * @param p The other point
	 * @return The distance
	 */
	double distanceTo(const geometry_msgs::Point &p) const;

	/// the time stamp of the associated positions
	ros::Time stamp;

	/// the GPS fix
	sensor_msgs::NavSatFix gpsFix;

	/// GPS position in the anchor frame
	geometry_msgs::Point gpsPosition;
};

class GpsKeyPoint : public GpsHint {
public:
	/// Initialize from hint
	GpsKeyPoint(const GpsHint &hint);

	/**
	 * Computes the estimated robot pose in the anchor frame, based on the
	 * given map alignment and the scaled SLAM base link.
	 * @param alignment The map alignment to be used
	 * @return The robot's estimated pose
	 */
	geometry_msgs::Pose getRobotPose(const alignment::Alignment &alignment) const;

	/// the associated SLAM base link (not scaled!)
	tf::Transform baseLink;
};

typedef std::vector<GpsHint> GpsHintVect;
typedef std::vector<GpsKeyPoint> GpsKeyPointVect;

class GpsManager;
typedef boost::shared_ptr<GpsManager> GpsManagerPtr;
typedef boost::shared_ptr<GpsManager const> GpsManagerConstPtr;

/**
 * # GPS Manager
 *
 * ## Published topics
 * - **gps_position_marker**: The GPS positions for visualization purposes
 * - **gps_hints_updated**: Signal that the GPS key points have been updated
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
	 * A GPS hint is considered out of date when its age (the difference between the
	 * current time and it's time stamp) is greater than the cache time of the tf listener.
	 * @param hint The GPS hint to be tested
	 * @return True if it is out of date, otherwise false
	 */
	bool hintIsOutdated(const GpsHint &hint);

	/**
	 * Converts a GPS fix to the known map anchor frame. If the GPS fix lies in a different UTM grid zone
	 * than the known map anchor an exception is thrown.
	 * @param gpsFix The GPS fix
	 * @param anchor The known map anchor GeoPose
	 * @return The position in the anchor frame
	 * @throws DifferentUTMGridZones
	 */
	static geometry_msgs::Point convertGPSPositionToAnchorFrame(const sensor_msgs::NavSatFix &gpsFix, const geographic_msgs::GeoPose &anchor);

	/**
	 * Publishes key point marker for visualization/debugging purposes.
	 */
	void publishKeyPointMarker();

	/**
	 * Publishes GPS fix marker for visualization/debugging purposes.
	 */
	void publishGpsFixMarker();

private:
	static GpsManagerPtr _instance;

	/// Transform listener
	tf::TransformListener listener;

	ros::WallTimer timer;

	/// Subscriber for the GPS fix topic
	ros::Subscriber gpsFixSubscriber;

	/// Publishes marker for visualization/debugging purposes
	ros::Publisher gpsPositionMarkerPublisher;

	/// Publishes an empty message as a signal that the GPS key points have been updated
	ros::Publisher gpsHintsUpdatedPublisher;

	/// queue of GPS fixes waiting to be paired with the according SLAM base links
	GpsHintVect hintQueue;

	/// GPS position/SLAM base link pairs
	GpsKeyPointVect keypoints;
};

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPSMANAGER_H_ */
