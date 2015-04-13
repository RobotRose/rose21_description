/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/10/22
*         - File copied and changed for rose21.
*
* Description:
*   Neck tf node, created from the construction paintings of the robot
* 
***********************************************************************************/
#include "rose21_neck_tf/neck_tf_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_neck");
    ros::NodeHandle n;
    ros::Rate rate(60);

    ROS_INFO_NAMED(ROS_NAME, "%s node started", ROS_NAME.c_str());

    TFNeck* tf_neck = new TFNeck(ROS_NAME, n);

    while(n.ok())
    {
        tf_neck->broadcast();
        ros::spinOnce();
        rate.sleep();
    }

    // Cleanup
    delete tf_neck;

    return 0;
}
