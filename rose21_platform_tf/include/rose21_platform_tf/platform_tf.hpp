/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
* Author: Okke Hendriks
* Date  : 2014/01/02
*     - File created.
*
* Description:
* This class contains the current pose of the platform aswell as the functionallity
*   which calculate the odometry and publishes the tf's accordingly.
* 
***********************************************************************************/

#ifndef PLATFORM_TF_HPP
#define PLATFORM_TF_HPP

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <map>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "rose_common/common.hpp"
 
#include "odometry/odometry.hpp"
#include "tf_helper/tf_helper.hpp"

#include "rose21_platform/wheelunit_states.h"

using namespace std;

class TFPlatform : public TFHelper
{
  public:
    TFPlatform(string transform_name, ros::NodeHandle n, string from_link, string to_link);
    ~TFPlatform();

    void CB_WheelUnitStates(const rose21_platform::wheelunit_states::ConstPtr& wheelunit_states);
    
    void publishOdometry();
    map<string, WheelUnit>& getWheelUnits();

    const geometry_msgs::Pose&      getPose();
    const geometry_msgs::Twist&     getVelocity();

    geometry_msgs::Pose     pose_;
    geometry_msgs::Twist    velocity_;
    Odometry*               odometry_;    

  private:
    ros::Publisher     odom_pub_;
    ros::Subscriber    wheelunit_states_sub_;

    map<string, WheelUnit> wheelunits_map_;

    ros::Time start_time_;
    ros::Time end_time_;

};



#endif // PLATFORM_TF_HPP
