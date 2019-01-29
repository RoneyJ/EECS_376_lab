#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "lab1"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 0.37; // 0.3048m/s speed command
    double yaw_rate = 0.5236; //0.5 rad/sec yaw rate command (one third of pi/2)
    double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds

    geometry_msgs::Twist twist_cmd;
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;   
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;

    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }

    //move forward 3 meters
    twist_cmd.linear.x=speed; //command to move forward
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    }

    //rotate 90 degrees CCW
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z= yaw_rate; //spin in place clockwise
    timer=0.0; //reset the timer
    while(timer<time_3_sec + 0.5) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    }


    //move forward 3 meters
    twist_cmd.linear.x=speed; //command to move forward
    twist_cmd.angular.z= 0.0;
    timer=0.0;
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    }

    //rotate 90 degrees CCW
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z= yaw_rate; //spin in place clockwise
    timer=0.0; //reset the timer
    while(timer<time_3_sec + 0.5) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    }

    //move forward 3 meters
    twist_cmd.linear.x=speed; //command to move forward
    twist_cmd.angular.z= 0.0;
    timer=0.0;
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    }

    //rotate 90 degrees CCW
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z= yaw_rate; //spin in place clockwise
    timer=0.0; //reset the timer
    while(timer<time_3_sec - 0.5) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    }

    //move forward 3 meters
    twist_cmd.linear.x=speed; //command to move forward
    twist_cmd.angular.z= 0.0;
    timer=0.0;
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    }

    //rotate 90 degrees CCW
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z= yaw_rate; //spin in place clockwise
    timer=0.0; //reset the timer
    while(timer<time_3_sec - 0.5) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    }
}
