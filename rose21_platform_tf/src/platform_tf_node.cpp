/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Okke Hendriks
*    Date  : 2014/01/02
*         - File created.
*
* Description:
*   This is a node that listens to the odometry messages of the wheel untis and 
*   publishes platform transforms accordingly.
* 
***********************************************************************************/

#include "rose21_platform_tf/platform_tf_node.hpp"

using namespace std;

#define ODOM_LOG
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_platform");
    ros::NodeHandle n;
    ros::Rate rate(70);

    ROS_INFO_NAMED(ROS_NAME, "%s node started", ROS_NAME.c_str());

 	// Create transform object
    TFPlatform* tf_platform = new TFPlatform(ROS_NAME, n, "/odom", "/base_link");

#ifdef ODOM_LOG
    // PID log files
    FILE * odom_log         = NULL;
    auto time_start_sec     = ros::Time::now().toSec();
    if(odom_log == NULL)
        odom_log = fopen("~/odom_log.dat", "w+");  
#endif

    // Keep on spinnin
    while(n.ok())
    {
#ifdef ODOM_LOG
        if(odom_log != NULL)
        {
            fprintf(odom_log, "%.2f", ros::Time::now().toSec() - time_start_sec);
            fprintf(odom_log, " %.2f %.2f %.2f %.2f"  
                                            , tf_platform->getWheelUnits().at("FL").getMeasuredAngleRad()
                                            , tf_platform->getWheelUnits().at("BL").getMeasuredAngleRad()
                                            , tf_platform->getWheelUnits().at("FR").getMeasuredAngleRad()
                                            , tf_platform->getWheelUnits().at("BR").getMeasuredAngleRad());
            fprintf(odom_log, " %.2f %.2f %.2f %.2f"  
                                            , tf_platform->getWheelUnits().at("FL").getMeasuredVelocityMetersPerSec()
                                            , tf_platform->getWheelUnits().at("BL").getMeasuredVelocityMetersPerSec()
                                            , tf_platform->getWheelUnits().at("FR").getMeasuredVelocityMetersPerSec()
                                            , tf_platform->getWheelUnits().at("BR").getMeasuredVelocityMetersPerSec());
            fprintf(odom_log, " %.2f %.2f %.2f %.2f %.2f %.2f"  
                                            , tf_platform->getPose().position.x
                                            , tf_platform->getPose().position.y
                                            , tf::getYaw(tf_platform->getPose().orientation)
                                            , tf_platform->getVelocity().linear.x
                                            , tf_platform->getVelocity().linear.y
                                            , tf_platform->getVelocity().angular.z);
            fprintf(odom_log, "\n");
        }    
#endif    
        ros::spinOnce();
        rate.sleep();
    }
    
    // Cleanup
    delete tf_platform;

#ifdef ODOM_LOG
    if(odom_log != NULL)
        fclose(odom_log);
#endif

    return 0;
}
