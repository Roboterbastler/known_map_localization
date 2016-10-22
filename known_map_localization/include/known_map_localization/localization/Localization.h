/*
 * Localization.h
 *
 *  Created on: 19.10.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_SRC_LOCALIZATION_LOCALIZATION_H_
#define KNOWN_MAP_LOCALIZATION_SRC_LOCALIZATION_LOCALIZATION_H_

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <filter/Filter.h>
#include <SlamScaleManager.h>

namespace kml {

/**
 * # Localization
 *
 * ## Parameters
 * - **localization_rate**: Determines the rate of localization (updates per second). Defaults to 10.
 *
 * ## Published topics
 * - **tf**: Base link from anchor frame to base_link frame
 * - **kml_base_link**: Base Link from anchor frame to base_link frame
 * - **geo_pose**:
 */
class Localization {
public:
	Localization(FilterConstPtr pFilter, SlamScaleManagerConstPtr pSlamScaleManager, KnownMapServerConstPtr pKnownMapServer);

	/**
	 * Check for transformation or alignment available.
	 */
	void localize();

	/**
	 * Computes the robot pose based on the given alignment and the SLAM base link at given time.
	 * The pose is given in the known map anchor frame.
	 * @param alignment The alignment used to compute the pose
	 * @return The robot pose, the stamp indicates the time stamp of the SLAM base link used
	 * @throws tf::TransformException If tf lookup of SLAM base link fails.
	 * @throws ScaleNotAvailable If no scale is available.
	 */
	tf::Stamped<tf::Pose> localizeWithAlignment(const Alignment &alignment, ros::Time time = ros::Time(0)) const;

private:
	void tick(const ros::WallTimerEvent& event);

	void publishGeoPose(const tf::StampedTransform &baseLink) const;

private:
	/// Used to regularly update position
	ros::WallTimer mTimer_;

	/// tf listener
	tf::TransformListener mListener_;

	/// Broadcaster for tf transforms
	tf::TransformBroadcaster mBroadcaster_;

	/// Publishes the base link
	ros::Publisher mBaseLinkPublisher_;

	/// Publishes the localization result as a geographic pose
	ros::Publisher mGeoPosePublisher_;

private:
	FilterConstPtr pFilter_;
	SlamScaleManagerConstPtr pSlamScaleManager_;
	KnownMapServerConstPtr pKnownMapServer_;
};

typedef boost::shared_ptr<Localization> LocalizationPtr;
typedef boost::shared_ptr<Localization const> LocalizationConstPtr;

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_SRC_LOCALIZATION_LOCALIZATION_H_ */
