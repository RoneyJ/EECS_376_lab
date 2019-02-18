// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <mobot_pub_des_state/path.h>
#include <std_srvs/Trigger.h>

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int start_ping_index_= -1; // NOT real; callback will have to find this
int end_ping_index_ = -1;
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
bool estop_ = false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
ros::ServiceClient estop_client_;
ros::ServiceClient clear_estop_client_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (start_ping_index_<=0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping on the left and on the right of the robot?
        start_ping_index_ = (int) (((-3.1415926/2) -angle_min_)/angle_increment_);
        end_ping_index_ = (int) ((3.1415926/2 - angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: start_ping_index = %d",start_ping_index_);
        ROS_INFO("LIDAR setup: end_ping_index = %d",end_ping_index_);
    }
   ROS_INFO("shortest ping dist in front = %f",ping_dist_in_front_);
   double angle = (-3.1415926/2);
   if(!estop_){
   for(int i = start_ping_index_; i < end_ping_index_; i++){

	   double MIN_SAFE_DISTANCE = -((3.1415926/2)*angle*angle)+1.5; //equation for min_safe_distance
	   angle = angle + angle_increment_;
	   ping_dist_in_front_ = laser_scan.ranges[i];
	   if (ping_dist_in_front_<MIN_SAFE_DISTANCE) {
		   ROS_WARN("DANGER, WILL ROBINSON!!");
		   laser_alarm_=true;
		   std_srvs::Trigger trigger_srv;
		   estop_client_.call(trigger_srv);
		   ros::Duration(5).sleep();
		   break;
	   }
	   else {
		   if(laser_alarm_){
			   std_srvs::Trigger trigger_srv;
			   clear_estop_client_.call(trigger_srv);
		   }
		   laser_alarm_=false;
	   }
   }
   }
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

void ESTOPCallback(const std_msgs::Bool& ESTOP_msg) {
	estop_ = ESTOP_msg.data;
    if(estop_){
	  std_srvs::Trigger trigger_srv;
      estop_client_.call(trigger_srv);
      ros::Duration(5).sleep();
    }else{
      std_srvs::Trigger trigger_srv;
	  clear_estop_client_.call(trigger_srv);
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
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
    ros::Subscriber ESTOP_subscriber = nh.subscribe("/ESTOP", 1, ESTOPCallback);
	bool eStop = false;

    ros::ServiceClient stop_client = nh.serviceClient<std_srvs::Trigger>("estop_service");
    ros::ServiceClient start_client = nh.serviceClient<std_srvs::Trigger>("clear_estop_service");
    estop_client_ = stop_client;
    clear_estop_client_ = start_client;
    while (!stop_client.exists()) {
    	ROS_INFO("waiting for service...");
    	ros::Duration(1.0).sleep();
    }
    while (!start_client.exists()) {
        	ROS_INFO("waiting for service...");
        	ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");



    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

