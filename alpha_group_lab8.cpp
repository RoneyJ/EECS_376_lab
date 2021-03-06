// alpha_group_lab8: 
// uses right arm trajectory action server to send robot to hard_coded poses to pick up a box, bring it towards the torso and return it

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <simple_baxter_gripper_interface/simple_baxter_gripper_interface.h>

#include <baxter_trajectory_streamer/trajAction.h>
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

int g_done_count=0;
void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_action_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        

    Eigen::VectorXd q_pre_grasp, q_grasp, q_stow;
    Eigen::VectorXd q_vec_right_arm;
    std::vector<Eigen::VectorXd> des_path_right;
    trajectory_msgs::JointTrajectory des_trajectory_right; // empty trajectory   
    
    BaxterGripper baxterGripper(&nh);

    //here are hard-coded joint angles for the right arm pose
    cout<<"setting pre-poses: "<<endl;
    q_pre_grasp.resize(7);
    q_grasp.resize(7);
    q_stow.resize(7);
    q_pre_grasp << 0.465, -0.176, 0.881, 1.192, -1.135, 1.018, 0.733; //1.31, 0.98, 0.436, 0.052, -1.56, 1.21, 0.652;
    q_grasp << 0.566, 0.05, 0.821, 0.897, -1.156, 0.942, 0.586;//1.29, 0.820, 0.489, 0.202, -1.62, 1.09, 0.571;
    q_stow << 0.524, 0.102, 0.929, 2.08, -2.06, 1.08, 0.437;//0.929, 2.08, 0.524, 0.102, -2.06, 1.08, 0.437;
    //cout<<"enter 1";
    //int ans;
    //cin>>ans;
    ROS_INFO("instantiating a traj streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();  //the baxter_traj_streamer needs spins for its updates
        ros::Duration(0.01).sleep();
    }

    //instantiate clients of the right arm server:
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> right_arm_action_client("rightArmTrajActionServer", true);

    // attempt to connect to the servers:
    ROS_INFO("waiting for right-arm server: ");
    bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on right-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;   

    while (baxterGripper.get_right_gripper_pos() < -0.1){			//<-0.5) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
        ROS_INFO("waiting for right gripper position filter to settle; pos = %f", baxterGripper.get_right_gripper_pos());
    }

    // goal objects compatible with the arm servers
	baxter_trajectory_streamer::trajGoal goal_right;

	int done_count = 0;

	//get current pose of right arm:    
	q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();

	while(ros::ok()){  
		des_path_right.clear();

		des_path_right.push_back(q_vec_right_arm); //start from current pose
		des_path_right.push_back(q_pre_grasp);

		//convert from vector of 7dof poses to trajectory message  
		baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
		    
		//  copy traj to goal:
		goal_right.trajectory = des_trajectory_right;  
		    
		ROS_INFO("sending goal to right arm: ");
		right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);  
		while (g_done_count - done_count<1) {
		   ROS_INFO("waiting to finish pre-pose..");
		   ROS_INFO("g_done_count = %d", g_done_count);
		   ros::Duration(1.0).sleep();
		}
		done_count++;
		ros::Duration(1.0).sleep();

	   
	    des_path_right.clear();

	    des_path_right.push_back(q_pre_grasp);
	    des_path_right.push_back(q_grasp);

	    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
	    goal_right.trajectory = des_trajectory_right;

	    ROS_INFO("sending goal to right arm: ");
	    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);  
	    while (g_done_count - done_count<1) {
	       ROS_INFO("waiting to finish pre-pose..");
	       ROS_INFO("g_done_count = %d", g_done_count);
	       ros::Duration(1.0).sleep();
	    }
	    done_count++;
	    ros::Duration(1.0).sleep();

	    baxterGripper.right_gripper_close();

	    ros::Duration(1.0).sleep();

	    des_path_right.clear();

	    des_path_right.push_back(q_grasp);
	    des_path_right.push_back(q_pre_grasp);

	    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
	    goal_right.trajectory = des_trajectory_right;

	    ROS_INFO("sending goal to right arm: ");
	    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);  
	    while (g_done_count - done_count<1) {
	       ROS_INFO("waiting to finish pre-pose..");
	       ROS_INFO("g_done_count = %d", g_done_count);
	       ros::Duration(1.0).sleep();
	    }
	    done_count++;
	    ros::Duration(1.0).sleep();

	    des_path_right.clear();

	    des_path_right.push_back(q_pre_grasp);
	    des_path_right.push_back(q_stow);

	    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
	    goal_right.trajectory = des_trajectory_right;

	    ROS_INFO("sending goal to right arm: ");
	    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);  
	    while (g_done_count - done_count<1) {
	       ROS_INFO("waiting to finish pre-pose..");
	       ROS_INFO("g_done_count = %d", g_done_count);
	       ros::Duration(1.0).sleep();
	    }
	    done_count++;
	    ros::Duration(1.0).sleep();

	    des_path_right.clear();

	    des_path_right.push_back(q_stow);
	    des_path_right.push_back(q_pre_grasp);

	    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
	    goal_right.trajectory = des_trajectory_right;

	    ROS_INFO("sending goal to right arm: ");
	    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);  
	    while (g_done_count - done_count<1) {
	       ROS_INFO("waiting to finish pre-pose..");
	       ROS_INFO("g_done_count = %d", g_done_count);
	       ros::Duration(1.0).sleep();
	    }
	    done_count++;
	    ros::Duration(1.0).sleep();

	    des_path_right.clear();

	    des_path_right.push_back(q_pre_grasp);
	    des_path_right.push_back(q_grasp);

	    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
	    goal_right.trajectory = des_trajectory_right;

	    ROS_INFO("sending goal to right arm: ");
	    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);  
	    while (g_done_count - done_count<1) {
	       ROS_INFO("waiting to finish pre-pose..");
	       ROS_INFO("g_done_count = %d", g_done_count);
	       ros::Duration(1.0).sleep();
	    }
	    done_count++;
	    ros::Duration(1.0).sleep();

	    baxterGripper.right_gripper_open();

	    ros::Duration(1.0).sleep();

	    des_path_right.clear();

	    des_path_right.push_back(q_grasp);
	    des_path_right.push_back(q_pre_grasp);

	    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
	    goal_right.trajectory = des_trajectory_right;

	    ROS_INFO("sending goal to right arm: ");
	    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);  
	    while (g_done_count - done_count<1) {
	       ROS_INFO("waiting to finish pre-pose..");
	       ROS_INFO("g_done_count = %d", g_done_count);
	       ros::Duration(1.0).sleep();
	    }
	    done_count++;
	    ros::Duration(1.0).sleep();

	    des_path_right.clear();

	    des_path_right.push_back(q_pre_grasp);
	    des_path_right.push_back(q_vec_right_arm);

	    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
	    goal_right.trajectory = des_trajectory_right;

	    ROS_INFO("sending goal to right arm: ");
	    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);  
	    while (g_done_count - done_count<1) {
	       ROS_INFO("waiting to finish pre-pose..");
	       ROS_INFO("g_done_count = %d", g_done_count);
	       ros::Duration(1.0).sleep();
	    }
	    done_count++;

    }
    return 0;
}

