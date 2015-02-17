/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/10/29
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef ARM_TF_NODE_HPP
#define ARM_TF_NODE_HPP

#include <iostream>
#include <ros/ros.h>
#include <stdio.h>

#include "rose_common/common.hpp"
 
#include "tf_helper/tf_helper.hpp"
 

#define TRANSFORM_NAME "tf_arm"

/**
 * Callback when the right arm wrist pose is published
 * @param pose The pose of the right arm wirst 
 */
void CB_right_arm_wrist(const geometry_msgs::Pose pose);

/**
 * Callback when the left arm wrist pose is published
 * @param pose The pose of the left arm wirst 
 */
void CB_left_arm_wrist(const geometry_msgs::Pose pose);

#endif // ARM_TF_NODE_HPP