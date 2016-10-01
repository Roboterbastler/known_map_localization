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
#include <known_map_server/KnownMapServer.h>
#include <gps/GpsHint.h>

namespace kml {

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
	GpsManager(KnownMapServerConstPtr pKnownMapServer);

	/**
	 * Gives read-only access to the stored GPS position/SLAM base link hints.
	 * @return Vector of hints
	 */
	const GpsHintVect& getGpsHints() const;

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
	void updateGpsHints(const ros::WallTimerEvent& event);

	/**
	 * Returns the SLAM base link for the requested time.
	 * @param t The requested time
	 * @return The transformation
	 */
	tf::StampedTransform getSlamBaseLink(ros::Time t);

	/**
	 * A GPS position is considered out of date when its age (the difference between the
	 * current time and it's time stamp) is greater than the cache time of the tf listener.
	 * @param pos The GPS position to be tested
	 * @return True if it is out of date, otherwise false
	 */
	bool gpsPositionIsOutdated(const GpsPosition &pos);

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
	void publishGpsHintMarker();

	/**
	 * Publishes GPS fix marker for visualization/debugging purposes.
	 */
	void publishGpsPositionMarker();

private:
	/// Transform listener
	tf::TransformListener mListener_;

	ros::WallTimer mTimer_;

	/// Subscriber for the GPS fix topic
	ros::Subscriber mGpsPositionSubscriber_;

	/// Publishes marker for visualization/debugging purposes
	ros::Publisher mGpsMarkerPublisher_;

	/// Publishes an empty message as a signal that the GPS hints have been updated
	ros::Publisher mGpsHintsUpdatedPublisher_;

	/// queue of GPS fixes waiting to be paired with the according SLAM base links
	GpsPositionVect mPositions_;

	/// GPS position/SLAM base link pairs
	GpsHintVect mHints_;

private:
	KnownMapServerConstPtr pKnownMapServer_;
};

typedef boost::shared_ptr<GpsManager> GpsManagerPtr;
typedef boost::shared_ptr<GpsManager const> GpsManagerConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_KNOWN_MAP_LOCALIZATION_GPSMANAGER_H_ */
