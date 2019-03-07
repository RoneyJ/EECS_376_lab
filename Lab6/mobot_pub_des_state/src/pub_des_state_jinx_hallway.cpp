//pub_des_state_jinx_hallway:
//commands the robot to move down the hallway of 2nd floor Glennan from the passenger elevator to the service elevator

#include <ros/ros.h>
#include <math.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

using namespace std;

std_msgs::Bool lidar_alarm_set;

const bool no_lidar_alarm = true;
const bool lidar_alarm = false;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    geometry_msgs::Quaternion quat;

    ros::Publisher pub = n.advertise<std_msgs::Bool>("lidar_flag", 1);
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    
 

    float start = 0.0;
    float end = 10.0; //32.0;

    for(double i = start; i <= end; i+=0.5){
	    ROS_INFO("iteration %f", i);
	    lidar_alarm_set.data = lidar_alarm;
	    pub.publish(lidar_alarm_set);
            pose.position.x = i;
	    pose.position.y = 0.0;
	    pose.position.z = 0.0; // let's hope so!
	    //quat = convertPlanarPhi2Quaternion(0);
	    //pose.orientation = quat;
	    pose_stamped.pose = pose;
	    path_srv.request.path.poses.push_back(pose_stamped);
    }

    lidar_alarm_set.data = no_lidar_alarm;
    pub.publish(lidar_alarm_set);
    quat = convertPlanarPhi2Quaternion(M_PI);
    pose.orientation = quat;
    path_srv.request.path.poses.push_back(pose_stamped);

    ros::Duration(5.0).sleep();

    for(double i = end; i >= start; i-=0.5){
	    ROS_INFO("iteration %f", i);
	    lidar_alarm_set.data = lidar_alarm;
	    pub.publish(lidar_alarm_set);
            pose.position.x = i;
	    pose.position.y = 0.0;
	    pose.position.z = 0.0; // let's hope so!
	    //quat = convertPlanarPhi2Quaternion(0);
	    //pose.orientation = quat;
	    pose_stamped.pose = pose;
	    path_srv.request.path.poses.push_back(pose_stamped);
    }
   
    
    client.call(path_srv);

    return 0;
}
