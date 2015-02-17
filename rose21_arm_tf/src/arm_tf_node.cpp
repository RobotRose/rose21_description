/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/10/29
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/
#include "arm_controllers/arm_tf_node.hpp"

TFHelper* tf_right_arm_wrist;
TFHelper* tf_left_arm_wrist;

int main(int argc, char** argv)
{
    ros::init(argc, argv, TRANSFORM_NAME);
    ros::NodeHandle n;
    ros::Rate rate(10);

    ROS_INFO_NAMED(TRANSFORM_NAME, "%s node started", TRANSFORM_NAME);

    // Create transform objects
    // TFHelper* tf_right_arm              = new TFHelper("right_arm", n, "/lift_link", "/right_arm");
    // tf_right_arm_wrist                  = new TFHelper("right_wrist", n, "/arms", "/right_arm_wrist");
    // TFHelper* tf_right_arm_gripper_tip  = new TFHelper("right_gripper", n, "/right_arm_wrist", "/right_arm_gripper");

    TFHelper* tf_left_arm               = new TFHelper("left_arm", n, "/lift_link", "/left_arm");
    tf_left_arm_wrist                   = new TFHelper("left_wrist", n, "/left_arm", "/left_arm_wrist");
    TFHelper* tf_left_arm_gripper_tip   = new TFHelper("left_gripper", n, "/left_arm_wrist", "/left_arm_gripper");

    TFHelper* tf_bihand                 = new TFHelper("arms", n, "/lift_link", "/arms");

    tf_bihand->setTransform(M_PI/2, 0.0, M_PI/2, 0.101, 0.0, 0.6078);
    // tf_bihand->setTransform(M_PI/2, 0.0, M_PI/2, 0.1865-0.075, 0.0, 0.6078);
    // tf_bihand->setTransform(M_PI/2, 0.0, M_PI/2, 0.101, 0.0, 0.44);
    // Pan has a fixed transform from the lift link
    // tf_right_arm->setTransform(M_PI/2, M_PI, M_PI/2, 0.101, -0.195, 0.44); // Arm has been mounted upside down
    // tf_right_arm_gripper_tip->setTransform(0.0, 0.0, 0.0, 0.155, 0.0, 0.0);
    // tf_right_arm->setTransform(M_PI/2, 0.0, M_PI/2, 0.19, -0.195, 0.44);

    tf_left_arm->setTransform(M_PI/2, M_PI, M_PI/2, 0.101, 0.200, 0.6078);
    // tf_left_arm->setTransform(M_PI/2, M_PI, M_PI/2, 0.1865-0.075, 0.200, 0.6078);
    tf_left_arm_gripper_tip->setTransform(0.0, 0.0, 0.0, 0.155, 0.0, 0.0);

    // ros::Subscriber right_arm_wrist_sub = n.subscribe("/right_arm/end_effector_pose", 1, &CB_right_arm_wrist);
    ros::Subscriber left_arm_wrist_sub  = n.subscribe("/left_arm/end_effector_pose", 1, &CB_left_arm_wrist);

    // Keep on spinnin
    while(n.ok())
    {
        tf_bihand->Broadcast();

        // tf_right_arm->Broadcast();
        // tf_right_arm_wrist->Broadcast();
        // tf_right_arm_gripper_tip->Broadcast();

        tf_left_arm->Broadcast();
        tf_left_arm_wrist->Broadcast();
        tf_left_arm_gripper_tip->Broadcast();

        ros::spinOnce();
        rate.sleep();
    }

    // Cleanup
    // delete tf_right_arm;
    // delete tf_right_arm_wrist;
    // delete tf_right_arm_gripper_tip;

    delete tf_left_arm;
    delete tf_left_arm_wrist;
    delete tf_left_arm_gripper_tip;

    return 0;
}

void CB_right_arm_wrist(const geometry_msgs::Pose pose)
{
    // ROS_DEBUG_NAMED(TRANSFORM_NAME, "CB_right_arm_wrist : (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    geometry_msgs::Vector3 rpy = rose_conversions::quaternionToRPY(pose.orientation);
    tf_right_arm_wrist->setTransform(rpy.x, rpy.y, rpy.z, pose.position.x, pose.position.y, pose.position.z);
}


void CB_left_arm_wrist(const geometry_msgs::Pose pose)
{
    // ROS_DEBUG_NAMED(TRANSFORM_NAME, "CB_left_arm_wrist: (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    geometry_msgs::Vector3 rpy = rose_conversions::quaternionToRPY(pose.orientation);
    tf_left_arm_wrist->setTransform(rpy.x, rpy.y, rpy.z, pose.position.x, pose.position.y, pose.position.z);
}
