/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/01/24
* 		- File created.
*
* Description:
*	Publishes the position of the laser scanner(s) 
* 
***********************************************************************************/

#include "laser/laser_tf_node.hpp"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, TRANSFORM_NAME);
    ros::NodeHandle n;
    ros::Rate rate(20);

    ROS_INFO_NAMED(TRANSFORM_NAME, "%s node started", TRANSFORM_NAME);

 	// Create transform object
    TFHelper* tf_laser = new TFHelper(TRANSFORM_NAME, n, "/base_link", "/laser");
    
    //X = Half the length of the base + the distance from the front of the base to the centerpoint of the laser. = (0.76 / 2) + 0.055
    tf_laser->setTransform(tf::createQuaternionFromRPY(0.0, 0.0, 0.0), tf::Vector3((0.76/2)+0.055, 0.0, 0.0));     

    // Keep on spinnin
    while(n.ok())
    {
        // Calculate the odomotry and broadcast it 
        tf_laser->Broadcast();

        ros::spinOnce();
        rate.sleep();
    }
    
    // Cleanup
    delete tf_laser;
        
    return 0;
}
