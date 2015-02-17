/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Okke Hendriks
*    Date  : 2013/12/16
*         - File created.
*
* Description:
*	This class contains the current pose of the platform aswell as the functionallity
*   which calculate the odometry and publishes the tf's accordingly.
* 
***********************************************************************************/

#include "rose21_platform_tf/platform_tf.hpp"


TFPlatform::TFPlatform(string transform_name, ros::NodeHandle n, string from_link, string to_link)
    : TFHelper(transform_name, n, from_link, to_link)
{
	odom_pub_ 				= n.advertise<nav_msgs::Odometry>("/odom", 50);	
    wheelunit_states_sub_ 	= n_.subscribe("/wheel_controller/wheelunit_states", 50, &TFPlatform::CB_WheelUnitStates, this);
    ROS_INFO_NAMED(transform_name_, "'%s' has subscribed to /wheel_controller/wheelunit_states", transform_name_.c_str());

    // Create the wheelunits and add them to a map for easier access later on
	WheelUnit wheel_unit_FL("FR", 0);
	WheelUnit wheel_unit_FR("FL", 2);
	WheelUnit wheel_unit_BL("BR", 4);
	WheelUnit wheel_unit_BR("BL", 6);
    wheelunits_map_.insert( std::pair<string, WheelUnit>(wheel_unit_FL.name_, wheel_unit_FL) );
 	wheelunits_map_.insert( std::pair<string, WheelUnit>(wheel_unit_FR.name_, wheel_unit_FR) );
 	wheelunits_map_.insert( std::pair<string, WheelUnit>(wheel_unit_BL.name_, wheel_unit_BL) );
 	wheelunits_map_.insert( std::pair<string, WheelUnit>(wheel_unit_BR.name_, wheel_unit_BR) );

    pose_.position.x     = 0.0;
    pose_.position.y     = 0.0;
    pose_.position.z     = 0.0;
    pose_.orientation    = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    velocity_.linear.x   = 0.0;
    velocity_.linear.y   = 0.0;
    velocity_.linear.z   = 0.0;
    velocity_.angular.x  = 0.0;
    velocity_.angular.y  = 0.0;
    velocity_.angular.z  = 0.0;

    // Create odometry object
    odometry_      = new Odometry();
}

TFPlatform::~TFPlatform()
{

}

const geometry_msgs::Pose& TFPlatform::getPose()
{
	return pose_;
}

const geometry_msgs::Twist& TFPlatform::getVelocity()
{
	return velocity_;
}


void TFPlatform::CB_WheelUnitStates(const rose_base_msgs::wheelunit_states::ConstPtr& wheelunit_states)
{
	// Transfer data to the newly created wheel units 
    wheelunits_map_.at("FR").measured_rotation_ 			= wheelunit_states->angle_FR;
	wheelunits_map_.at("FL").measured_rotation_ 			= wheelunit_states->angle_FL;
	wheelunits_map_.at("BR").measured_rotation_ 			= wheelunit_states->angle_BR;
	wheelunits_map_.at("BL").measured_rotation_ 			= wheelunit_states->angle_BL;
	wheelunits_map_.at("FR").measured_velocity_ 			= wheelunit_states->velocity_FR;
	wheelunits_map_.at("FL").measured_velocity_ 			= wheelunit_states->velocity_FL;
	wheelunits_map_.at("BR").measured_velocity_ 			= wheelunit_states->velocity_BR;
	wheelunits_map_.at("BL").measured_velocity_ 			= wheelunit_states->velocity_BL;
	wheelunits_map_.at("FR").measured_drive_encoder_diff_ 	= wheelunit_states->diff_FR; 
	wheelunits_map_.at("FL").measured_drive_encoder_diff_ 	= wheelunit_states->diff_FL;
	wheelunits_map_.at("BR").measured_drive_encoder_diff_ 	= wheelunit_states->diff_BR;
	wheelunits_map_.at("BL").measured_drive_encoder_diff_ 	= wheelunit_states->diff_BL; 
	wheelunits_map_.at("FR").dT_ 				 			= wheelunit_states->dT_FR;
	wheelunits_map_.at("FL").dT_ 				 			= wheelunit_states->dT_FL;
	wheelunits_map_.at("BR").dT_ 				 			= wheelunit_states->dT_BR;
	wheelunits_map_.at("BL").dT_ 				 			= wheelunit_states->dT_BL;

    // Calculate the odometry and broadcast it
    if(odometry_->calculateOdometry(wheelunits_map_, pose_, velocity_))
        publishOdometry();


	end_time_ = ros::Time::now();
	ros::Duration d = end_time_ - start_time_; 
	//ROS_INFO_NAMED(transform_name_, "CB_WheelUnitStates rate: %.2f", 1.0/d.toSec());	
	start_time_ = ros::Time::now();
}

void TFPlatform::publishOdometry()
{
	ROS_DEBUG_NAMED(transform_name_, "Publishing odometry");

	// Set and publish transform
    setTransform(0.0, 0.0, tf::getYaw(pose_.orientation), pose_.position.x, pose_.position.y, 0.0);
    Broadcast(); 

	// The odometry is also publish to the 'odom' topic because this is need by the navigation stack
	// The navigation stack also needs the velocity and this is not included in the transforms
	// Since all odometry is 6DOF we'll need a quaternion created from yaw
	nav_msgs::Odometry odom;
	odom.header.stamp 			= ros::Time::now();
	odom.header.frame_id 		= from_link_;
	odom.child_frame_id 		= to_link_;
	// Set the position
	odom.pose.pose 	 			= pose_;
	// Set the velocity
	odom.twist.twist 			= velocity_;


	// Simply use identiy matrix for covariances for now (http://answers.ros.org/question/12642/calculating-covariances-of-robots-odometry/)

	odom.pose.covariance  = boost::assign::list_of 	(1)   (0)   (0)  (0)  (0)  (0)
                                                	(0)   (1)   (0)  (0)  (0)  (0)
                                                	(0)   (0)   (1)  (0)  (0)  (0)
                                                	(0)   (0)   (0)  (1)  (0)  (0)
                                                	(0)   (0)   (0)  (0)  (1)  (0)
                                                	(0)   (0)   (0)  (0)  (0)  (1) ;

    odom.twist.covariance = boost::assign::list_of (1) 	 (0)   (0)  (0)  (0)  (0)
                                                   (0) 	 (1)   (0)  (0)  (0)  (0)
                                                   (0)   (0)   (1)  (0)  (0)  (0)
                                                   (0)   (0)   (0)  (1)  (0)  (0)
                                                   (0)   (0)   (0)  (0)  (1)  (0)
                                                   (0)   (0)   (0)  (0)  (0)  (1) ;	
  

	odom_pub_.publish(odom);

	ROS_DEBUG_THROTTLE_NAMED(0.1, transform_name_, "Pos    x: %.3fm   y: %.3fm   th: %.3frad, %.3fdeg", pose_.position.x, pose_.position.y, tf::getYaw(pose_.orientation), tf::getYaw(pose_.orientation)*(180.0/M_PI));
	ROS_DEBUG_THROTTLE_NAMED(0.1, transform_name_, "Vel    x: %.3fm/s y: %.3fm/s th: %.3frad/s", velocity_.linear.x, velocity_.linear.y, velocity_.angular.z);
}


map<string, WheelUnit>& TFPlatform::getWheelUnits()
{
	return wheelunits_map_;
}
