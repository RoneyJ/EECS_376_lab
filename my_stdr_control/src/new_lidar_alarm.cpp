// wsn example program to illustrate LIDAR processing.  1/23/15
//Edited by Josh Roney(jpr87)

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <cmath>
#include <math.h>
#include <std_srvs/Trigger.h>

const double MIN_SAFE_DISTANCE = 2.0; // set alarm if anything is within 0.5m of the front of robot
const double MIN_SAFE_WIDTH = 0.3; 

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
int num_points = 0;
bool laser_alarm_=false;
bool srvcall = false;
int index_diff = 0; // difference from ping_index needed to be checked (for the width and height)

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
ros::Publisher lidar_index_publisher_;

//Service clients for the e-stop service
ros::ServiceClient estop_client_;
std_srvs::Trigger estop_srv_;

ros::ServiceClient estop_clear_client_;
std_srvs::Trigger estop_clear_srv_;

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        num_points = (int) ((angle_max_ - angle_min_) / angle_increment_);
        index_diff = (int) (1.57 / angle_increment_); // 90 degrees in terms of indeces
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        ROS_INFO("90 degs in indeces = %i", index_diff);
        
    }
    //Edit for PS2
    //Scans every point in 180 degree arc in front and makes sure every ping is out of the minimum safe distance or width
    //if any point is less than both the minimum safe distance and width, sound the alarm
    bool temp_alarm = false;
    int temp_index = ping_index_;
   	
    for(int i = (ping_index_ - index_diff); ((temp_alarm == false) && (i <= (ping_index_ + index_diff))) ; i += 10){
	float index_angle = abs(ping_index_ - i) * angle_increment_; // the angle, wrt straight ahead
	float dist = cos(index_angle) * laser_scan.ranges[i];
	float width = sin(index_angle) * laser_scan.ranges[i]; 
	if(dist < MIN_SAFE_DISTANCE && width < MIN_SAFE_WIDTH){
   	   temp_alarm = true;
   	   temp_index = i;
   	}
    }
    laser_alarm_ = temp_alarm;
   
   
    ping_dist_in_front_ = laser_scan.ranges[ping_index_];
    ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   
    // publish alarm
    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
   
    // publish distance to alarming point
    std_msgs::Float32 lidar_ind_msg;
    lidar_ind_msg.data = ping_index_ - temp_index;
    lidar_index_publisher_.publish(lidar_ind_msg); 

    //**Edit for PS4**
    //trigger estop when lidar alarm is triggered
    //clear the estop when lidar alarm is clear
    if(!laser_alarm_ && srvcall){
	estop_clear_client_.call(estop_clear_srv_);
	srvcall = false;
	ROS_WARN("Path clear: cancelling e-stop");
    }

    if(laser_alarm_ && !srvcall){
	estop_client_.call(estop_srv_);
	srvcall = true;
	ROS_WARN("Obstacle detected! E-stop triggered!");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Publisher pub3 = nh.advertise<std_msgs::Float32>("lidar_index", 1);  
    lidar_index_publisher_ = pub3;
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);

    estop_client_ = nh.serviceClient<std_srvs::TriggerRequest>("estop_service");
    estop_clear_client_ = nh.serviceClient<std_srvs::TriggerRequest>("clear_estop_service");

    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}
