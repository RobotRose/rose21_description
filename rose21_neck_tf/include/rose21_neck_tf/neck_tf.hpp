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
#ifndef NECK_TF_HPP
#define NECK_TF_HPP

#include <dynamixel_msgs/JointState.h>
#include <stdio.h>
#include <string>

#include "tf_helper/tf_helper.hpp"
 

using std::string;

class TFNeck
{
  public:
    TFNeck(string transform_name, ros::NodeHandle nzzz);
    ~TFNeck();

    void broadcast();

  private:
    void CB_pan_state(const dynamixel_msgs::JointState& state);
    void CB_tilt_state(const dynamixel_msgs::JointState& state);

    TFHelper* tf_pan_mount_;
    TFHelper* tf_pan_rotation_;
    TFHelper* tf_tilt_mount_;
    TFHelper* tf_tilt_rotation_;
    TFHelper* tf_camera_center_;
    TFHelper* tf_camera_;

    ros::Subscriber pan_state_sub_;
    ros::Subscriber tilt_state_sub_;

    ros::NodeHandle n_;
    string          name_;
};

#endif // NECK_TF_HPP
