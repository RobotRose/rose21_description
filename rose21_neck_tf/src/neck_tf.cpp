/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/10/23
* 		- File created.
*
* Description:
*	Neck TF
* 
***********************************************************************************/
#include "rose21_neck_tf/neck_tf.hpp"

TFNeck::TFNeck(string name, ros::NodeHandle n)
    : n_ (n)
    , name_ (name)
{
    // Create all transform objects
    // tf_pan_mount_     = new TFHelper("neck_mount",         n, "/lift_link",          "/neck_mount");
    // tf_pan_rotation_  = new TFHelper("neck_pan",           n, "/neck_mount",         "/neck_pan");
    // tf_tilt_mount_    = new TFHelper("neck_tilt_mount",    n, "/neck_pan",           "/neck_tilt_mount");
    // tf_tilt_rotation_ = new TFHelper("neck_tilt",          n, "/neck_tilt_mount",    "/neck_tilt");
    // tf_camera_center_ = new TFHelper("neck_camera_center", n, "/neck_tilt",          "/camera_center_link");
    tf_camera_        = new TFHelper("neck_camera",        n, "/camera_center_link", "/rose/camera_link");

    double pan_mount_x;
    double pan_mount_y;
    double pan_mount_z;

    double tilt_mount_x;
    double tilt_mount_y;
    double tilt_mount_z;

    // Default values are the ones of Rose Akia
    n_.param("/neck_tf/pan_mount_x", pan_mount_x, 0.077);
    n_.param("/neck_tf/pan_mount_y", pan_mount_y, 0.0);
    n_.param("/neck_tf/pan_mount_z", pan_mount_z, 0.790654);

    n_.param("/neck_tf/tilt_mount_x", tilt_mount_x, 0.2805);
    n_.param("/neck_tf/tilt_mount_y", tilt_mount_y, 0.0);
    n_.param("/neck_tf/tilt_mount_z", tilt_mount_z, 0.031008);

    // Set (X,Y,Z) transformations
    // tf_pan_mount_->    setTransform(0.0, 0.0, 0.0, pan_mount_x,  pan_mount_y,  pan_mount_z ); // X- and Z-Transform between /lift_link and pan rotation point
    // tf_tilt_mount_->   setTransform(0.0, 0.0, 0.0, tilt_mount_x, tilt_mount_y, tilt_mount_z ); // Transformation between pan point and tilt point
    
    // tf_camera_center_->setTransform(0.0, 0.0, 0.0, 0.0   , 0.0 , 0.0659   ); // Z-transformation between tilt rotation point and enter point of camera in Kinect
    tf_camera_->       setTransform(0.0, 0.0, 0.0, 0.03  , 0.05, 0.0      ); // Camera location on the kinect

    // Initialize rotations at 0 (assumption)
    // tf_pan_rotation_-> setTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // tf_tilt_rotation_->setTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // pan_state_sub_   = n_.subscribe("/neck_pan_controller/state" , 100, &TFNeck::CB_pan_state, this);
    // tilt_state_sub_  = n_.subscribe("/neck_tilt_controller/state", 100, &TFNeck::CB_tilt_state, this);
}

TFNeck::~TFNeck()
{
    // Cleanup pointers
    delete tf_pan_mount_;
    delete tf_pan_rotation_;
    delete tf_tilt_mount_;
    delete tf_tilt_rotation_;
    delete tf_camera_center_;
    delete tf_camera_;
}

void TFNeck::broadcast()
{
    // Broadcast all TFs
    // tf_pan_mount_->Broadcast();
    // tf_pan_rotation_->Broadcast();
    // tf_tilt_mount_->Broadcast();
    // tf_tilt_rotation_->Broadcast();
    // tf_camera_center_->Broadcast();
    tf_camera_->Broadcast();
}

void TFNeck::CB_pan_state(const dynamixel_msgs::JointState& state)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_pan_state: %.3f", state.current_pos);
    tf_pan_rotation_->setTransform(0.0, 0.0, state.current_pos, 0.0, 0.0, 0.0);
}

void TFNeck::CB_tilt_state(const dynamixel_msgs::JointState& state)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_tilt_state: %.3f", state.current_pos);
    tf_tilt_rotation_->setTransform(0.0, -state.current_pos, 0.0, 0.0, 0.0, 0.0);
}
