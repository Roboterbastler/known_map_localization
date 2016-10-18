/*
 * GpsFilter.cpp
 *
 *  Created on: 11.09.2016
 *      Author: jacob
 */

#include <filter/GpsFilter.h>

#include <Exception.h>
#include <Utils.h>

#define MAX_HINT_CACHE_SIZE 10

namespace kml {

GpsFilter::GpsFilter(GpsManagerConstPtr pGpsManager,
		KnownMapServerConstPtr pKnownMapServer,
		SlamScaleManagerPtr pSlamScaleManager,
		StatusPublisherPtr pStatusPublisher, DataLoggerPtr pDataLogger) :
		Filter(pSlamScaleManager, pStatusPublisher, pDataLogger), pGpsManager_(pGpsManager), pKnownMapServer_(
				pKnownMapServer) {
	ROS_ASSERT(pGpsManager_);
	ROS_ASSERT(pKnownMapServer_);
	ROS_ASSERT(pSlamScaleManager_);

	ROS_INFO("    Type: GPS filter");

	ros::NodeHandle nh("~");

	mConstraintsMarker_ = setUpContraintMarker();
	mGpsConstraintsMarkerPublisher_ = nh.advertise<visualization_msgs::Marker>(
			"gps_position_marker", 1);

	kGpsConstraintRadius_ = nh.param("gps_constraint_radius", 10.);
	kDecayFactor_ = nh.param("aging_rate", 1.);
	kConfirmationFactor_ = nh.param("gps_confirmation_factor", 1.05);
	kPreferGpsSupported_ = nh.param("always_prefer_gps_supported", true);

	ROS_INFO("    Constraint radius: %.2f", kGpsConstraintRadius_);
	ROS_INFO("    Aging rate: %.2f", kDecayFactor_);
	ROS_INFO("    Always prefer GPS supported: %s", kPreferGpsSupported_ ? "true" : "false");
}

void GpsFilter::addHypotheses(const HypothesesVect &hypotheses) {
	bool filteredHypothesisModified = false;

	if (mReady_) {
		// small degradation factor
		mFilteredGpsHypothesis_.score *= kDecayFactor_;
	}

	ROS_DEBUG(
			"Checking %ld new hypotheses. Current filtered alignment score: %f",
			hypotheses.size(), mFilteredGpsHypothesis_.score);

	// score all new hypotheses
	for (HypothesesVect::const_iterator h = hypotheses.begin();
			h != hypotheses.end(); ++h) {
		visualization_msgs::Marker hypothesisConstraints =
				setUpContraintMarker();

		GpsScoredHypothesis scoredHypothesis(*h);

		// try to validate hypothesis by checking GPS hints
		validateHypothesis(scoredHypothesis, hypothesisConstraints);

		ROS_DEBUG("      -> Hypothesis has score: %f", scoredHypothesis.score);

		if (scoredHypothesis.score > mFilteredGpsHypothesis_.score) {
			// always prefer GPS-supported hypotheses
			if (!mFilteredGpsHypothesis_.gpsSupported
					|| scoredHypothesis.gpsSupported || !kPreferGpsSupported_) {
				mFilteredGpsHypothesis_ = scoredHypothesis;
				filteredHypothesisModified = true;
				mReady_ = true;

				if(mFilteredGpsHypothesis_.gpsSupported) {
					pStatusPublisher_->setStatus(STATUS_VALIDATED_POS, mFilteredGpsHypothesis_.supportingHints);
				} else {
					pStatusPublisher_->setStatus(STATUS_POS);
				}

				mConstraintsMarker_ = hypothesisConstraints;
				ROS_DEBUG("        -> New best alignment.");
			} else {
				ROS_DEBUG(
						"        -> Rejected, because current filtered hypothesis is supported by GPS hints.");
			}
		}
	}

	if (filteredHypothesisModified) {
		logAlignment(mFilteredGpsHypothesis_);
	}

	mGpsConstraintsMarkerPublisher_.publish(mConstraintsMarker_);
}

const Alignment& GpsFilter::getAlignment() const {
	if (!mReady_) {
		throw AlignmentNotAvailable("Filtered hypothesis is not yet available");
	}
	return mFilteredGpsHypothesis_;
}

geometry_msgs::Pose GpsFilter::estimatedRobotPose(const Alignment &alignment,
		const GpsHint &hint) const {
	tf::Transform slamMapFrame_to_knownMapFrame = alignment.toTfTransform();

	// convert pose to real world scale
	tf::Transform slamMapFrame_to_slamBaseLink =
			pSlamScaleManager_->convertTransform(hint.baseLink);

	tf::Pose pose;
	pose.setIdentity();
	pose *= slamMapFrame_to_knownMapFrame.inverse();
	pose *= slamMapFrame_to_slamBaseLink;

	geometry_msgs::Pose poseMsg;
	tf::poseTFToMsg(pose, poseMsg);
	return poseMsg;
}

void GpsFilter::validateHypothesis(GpsScoredHypothesis &h,
		visualization_msgs::Marker &constraints) const {
	try {
		const GpsHintVect &gpsHints = pGpsManager_->getGpsHints();
		for (GpsHintVect::const_iterator hint = gpsHints.begin();
				hint != gpsHints.end(); ++hint) {
			geometry_msgs::Pose robotPose = estimatedRobotPose(h, *hint);
			float d = kml::distance(hint->gpsPosition, robotPose.position);
			bool supportingConstraint = d < kGpsConstraintRadius_;
			float confirmation =
					supportingConstraint ?
							kConfirmationFactor_ : 1.0 / kConfirmationFactor_;

			h.score = confirmation * h.score;
			h.gpsSupported |= supportingConstraint;

			if(supportingConstraint) {
				h.supportingHints++;
			} else {
				h.challengingHints++;
			}

			addConstraintMarker(*hint, constraints, supportingConstraint);

			ROS_DEBUG(
					"    - Distance: %f  Confirmation: %f  Confirmed Score: %f",
					d, confirmation, h.score);
		}
	} catch (ScaleNotAvailable &e) {
		ROS_DEBUG("Scoring not possible: %s", e.what());
	} catch (tf::TransformException &e) {
		ROS_DEBUG("Scoring not possible: tf lookup failed (%s)", e.what());
	}
}

visualization_msgs::Marker GpsFilter::setUpContraintMarker() const {
	visualization_msgs::Marker marker;
	marker.header.frame_id = pKnownMapServer_->getKnownMap()->header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "GPS-Constraints";
	marker.id = 2;
	marker.frame_locked = true;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = kGpsConstraintRadius_;
	marker.scale.y = kGpsConstraintRadius_;
	marker.scale.z = kGpsConstraintRadius_;
	marker.pose.orientation.w = 1.0;
	marker.color.a = 0.5;
	return marker;
}

void GpsFilter::addConstraintMarker(const GpsHint &hint,
		visualization_msgs::Marker &marker, bool supporting) const {
	std_msgs::ColorRGBA color;
	color.a = 0.5;
	if (supporting) {
		color.g = 1.;
	} else {
		color.r = 1.;
	}
	marker.colors.push_back(color);
	marker.points.push_back(hint.gpsPosition);
}

} /* namespace kml */
