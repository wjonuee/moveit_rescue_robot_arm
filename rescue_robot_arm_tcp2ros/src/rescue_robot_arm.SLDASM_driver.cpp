#include <ros/ros.h>
#include <std_msgs/String.h>

//Publish
#include <sensor_msgs/JointState.h>

//Subscribe
//#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <control_msgs/FollowJointTrajectoryResult.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>

//#include <actionlib/server/simple_action_server.h>	//Actionlib

#include "Command.h"
#include "Connect.h"

#include <iostream>

using namespace std;

typedef struct _ARM
{
	double arm_ptz;
	double arm_joint1;
	double arm_joint2;
	double arm_stretch;
	double arm_pitch;
	double arm_roll;
	double arm_gripper;
} ARM_JOINT;

//m_connect = new Connection;	
Connection* m_connect = new Connection;


//typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> Server;

/*void arm_command_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
	ARM_JOINT arm_goal;

	goal->points.positions[0] = arm_goal.arm_ptz;
	goal->points.positions[1] = arm_goal.arm_joint1;
	goal->points.positions[2] = arm_goal.arm_joint2;
	goal->points.positions[3] = arm_goal.arm_pitch;
	goal->points.positions[4] = arm_goal.arm_roll;
	goal->points.positions[5] = arm_goal.arm_gripper;

	char goal_position[150];
	sprintf(goal_position,"ARMSETV3 {Type Direct} {Name Position} {PTZ %f} {Joint1 %f} {Joint2 %f} {Stretch %f} {Pitch %f} {Roll %f} {Gripper %f}\r\n",arm_goal.arm_ptz, arm_goal.arm_joint1, arm_goal.arm_joint2, arm_goal.arm_stretch, arm_goal.arm_pitch, arm_goal.arm_roll, arm_goal.arm_gripper);
	string goal_position_str(goal_position);
	
	m_connect->sendMsg(goal_position_str);
	cout<<goal_position_str<<endl;
}*/
void arm_command_callback(const sensor_msgs::JointStateConstPtr& arm_joint_states);

void arm_command_callback(const sensor_msgs::JointStateConstPtr& arm_joint_states)
{
	ARM_JOINT arm_end_position;

	//arm_joint_states->position.resize(6);
	arm_end_position.arm_ptz = arm_joint_states->position[0];
	arm_end_position.arm_joint1 = arm_joint_states->position[1];
	arm_end_position.arm_joint2 = arm_joint_states->position[2];
	arm_end_position.arm_pitch = arm_joint_states->position[3];
	arm_end_position.arm_roll = arm_joint_states->position[4];
	arm_end_position.arm_gripper = arm_joint_states->position[5];

	arm_end_position.arm_stretch = 0;

	char end_position[255];
	sprintf(end_position,"ARMSETV3 {Type Direct} {Name Position} {PTZ %f} {Joint1 %f} {Joint2 %f} {Stretch %f} {Pitch %f} {Roll %f} {Gripper %f}\r\n",arm_end_position.arm_ptz, arm_end_position.arm_joint1, arm_end_position.arm_joint2, arm_end_position.arm_stretch, arm_end_position.arm_pitch, arm_end_position.arm_roll, arm_end_position.arm_gripper);
	string end_position_str(end_position);
	
	//m_connect->sendMsg(end_position_str);
	cout<<"Command:"<<end_position_str<<endl;
}

int main(int argc, char ** argv)
{
	

	ros::init(argc, argv, "tcp2ros_driver");
	ros::NodeHandle n;
	//ros::Publisher arm_feedback_pub = n.advertise<std_msgs::String>("arm_feedback", 1000);
	ros::Publisher joint_state_pub  = n.advertise<sensor_msgs::JointState>("/robot/joint_states",1000);
	
	ros::Subscriber arm_command_sub = n.subscribe("/joint_states",1000,arm_command_callback);
	//Server server(n, "rescue_robot_arm_controller/follow_joint_trajectory", boost::bind(&arm_command_callback, &server ,_1), false);
	//server.start();
	
	

	//Command* m_command;
	//m_command = new Command;

	if(m_connect->Oninit("192.168.1.114",10000))
	{
		cout<<"connect success!"<<endl;
		m_connect ->sendMsg("REG {Mode WR} {Period 100}\r\n");
	}
	else
	{
		cout<<"connect failed!"<<endl;
	}
		
	char* arm_feedback_rcvbuffer;
	arm_feedback_rcvbuffer = new char[buffer_size];

	ARM_JOINT feedback;

	feedback.arm_ptz = 0;
	feedback.arm_joint1 = 0;
	feedback.arm_joint2 = 0;
	feedback.arm_pitch = 0;
	feedback.arm_roll = 0;
	feedback.arm_gripper = 0;
	
	feedback.arm_stretch = 0;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	//long count = 0;
	
	ros::Rate loop_rate(10.0);
	while(n.ok())
	{
		//current_time = ros::Time::now();
		//bzero(m_connect->rcvbuffer, buffer_size);
		if(m_connect->recvMsg() != -1)
		{
			ROS_INFO("receive arm feedback data");
			//cout<<m_connect->rcvbuffer;
		}
		else
		{
			ROS_INFO("receive arm feedback data ERROR!");
		}

		if(m_connect->recv_size > 0)
		{
			bzero(arm_feedback_rcvbuffer, buffer_size);
			memmove(arm_feedback_rcvbuffer, m_connect->rcvbuffer, m_connect->recv_size);
			string str(arm_feedback_rcvbuffer);
			//cout<<arm_feedback_rcvbuffer;

			int p1, p2;
			string temp;
			
			/* feedback arm position msg formate:
			*  SEN {Type Arm} {Name Position} {PTZ 0} {Joint1 0} {Joint2 0} {Stretch 0} {Pitch 0} {Roll 0} {Gripper 0}
			*/
			
			p1 = str.find("SEN {Type Arm} {Name Position}");
			if(p1 < 0)
			{
				ROS_INFO("can't find arm feedback data");
				//continue;
			}
			else
			{
				p1 = str.find("{PTZ" , p1);
				p2 = str.find("}", p1);
				temp = str.substr(p1 + 5, p2 - p1 -5);  //5 = szieof("{PTZ ");
				feedback.arm_ptz = atof(temp.c_str());///1000.0;
			
				p1 = str.find("{Joint1" , p2);
				p2 = str.find("}", p1);
				temp = str.substr(p1 + 8, p2 - p1 -8);
				feedback.arm_joint1 = atof(temp.c_str());///1000.0;

				p1 = str.find("{Joint2" , p2);
				p2 = str.find("}", p1);
				temp = str.substr(p1 + 8, p2 - p1 -8);
				feedback.arm_joint2 = atof(temp.c_str());///1000.0;

				p1 = str.find("{Stretch" , p2);
				p2 = str.find("}", p1);
				temp = str.substr(p1 + 8, p2 - p1 -8);
				feedback.arm_stretch = atof(temp.c_str());///1000.0;
			
				p1 = str.find("{Pitch" , p2);
				p2 = str.find("}", p1);
				temp = str.substr(p1 + 7, p2 - p1 -7);
				feedback.arm_pitch = atof(temp.c_str());///1000.0;

				p1 = str.find("{Roll" , p2);
				p2 = str.find("}", p1);
				temp = str.substr(p1 + 6, p2 - p1 -6);
				feedback.arm_roll = atof(temp.c_str());///1000.0;

				p1 = str.find("{Gripper" , p2);
				p2 = str.find("}", p1);
				temp = str.substr(p1 + 9, p2 - p1 -9);
				feedback.arm_gripper = atof(temp.c_str());///1000.0;
			}
		}
		else
		{
			ROS_INFO("receive  empty arm feedback data");
		}
	
		ros::Time arm_feedback_msg_time = ros::Time::now();
	
		sensor_msgs::JointState arm_feedback_msg;
		arm_feedback_msg.header.stamp = arm_feedback_msg_time;
		
		arm_feedback_msg.name.resize(6);
		arm_feedback_msg.name[0] = "ARM_PTZ_Joint";
		arm_feedback_msg.name[1] = "ARM_Joint1";
		arm_feedback_msg.name[2] = "ARM_Joint2";
		arm_feedback_msg.name[3] = "ARM_PITCH_Joint";
		arm_feedback_msg.name[4] = "ARM_ROLL_Joint";
		arm_feedback_msg.name[5] = "ARM_GRIPPER_Joint";

		arm_feedback_msg.position.resize(6);
		arm_feedback_msg.position[0] = feedback.arm_ptz;
		arm_feedback_msg.position[1] = feedback.arm_joint1;
		arm_feedback_msg.position[2] = feedback.arm_joint2;
		arm_feedback_msg.position[3] = feedback.arm_pitch;
		arm_feedback_msg.position[4] = feedback.arm_roll;
		arm_feedback_msg.position[5] = feedback.arm_gripper;
		
		arm_feedback_msg.velocity.resize(6);
		for( int i = 0; i < 6; i++)
			arm_feedback_msg.velocity[i] = 0;
		
		char feebback_joint_position[255];
		sprintf(feebback_joint_position,"SEN {Type ARM} {Name Position} {PTZ %f} {Joint1 %f} {Joint2 %f} {Stretch %f} {Pitch %f} {Roll %f} {Gripper %f}\r\n",arm_feedback_msg.position[0], arm_feedback_msg.position[1], arm_feedback_msg.position[2], 0.0, arm_feedback_msg.position[3], arm_feedback_msg.position[4], arm_feedback_msg.position[5]);
		string feebback_joint_position_str(feebback_joint_position);

		cout<<"Feedback:"<<feebback_joint_position_str<<endl;
		
		//for( int i = 0; i < 6; i++)
			//ROS_INFO("arm_joint%d_position:%f",i,arm_feedback_msg.position[i]);

		joint_state_pub.publish(arm_feedback_msg);

		ros::spinOnce();
		loop_rate.sleep();
		//++count;
	}

	return 0;
}
