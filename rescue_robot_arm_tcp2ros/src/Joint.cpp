#include "Joint.h"

Joint::Joint()
{
	arm_real_joint->arm_ptz = 0.0;
	arm_real_joint->arm_joint1 = 0.0;
	arm_real_joint->arm_joint2 = 0.0;
	arm_real_joint->arm_stretch = 0.0;
	arm_real_joint->arm_pitch = 0.0;
	arm_real_joint->arm_roll = 0.0;
	arm_real_joint->arm_gripper = 0.0;
	
	arm_moveit_joint->arm_ptz = 0.0;
	arm_moveit_joint->arm_joint1 = 0.0;
	arm_moveit_joint->arm_joint2 = 0.0;
	arm_moveit_joint->arm_stretch = 0.0;
	arm_moveit_joint->arm_pitch = 0.0;
	arm_moveit_joint->arm_roll = 0.0;
	arm_moveit_joint->arm_gripper = 0.0;
}

Joint::~Joint()
{
	
}

double Joint::arm_PTZ_real_2_moveit()
{
	return arm_moveit_joint->arm_ptz = 0.0;
}

double Joint::arm_PTZ_moveit_2_real()
{
	return arm_real_joint->arm_ptz = 0.0;
}

double Joint::arm_Joint1_real_2_moveit()
{
	arm_moveit_joint->arm_joint1 = (-3.32/1520.0)*arm_real_joint->arm_joint1 + 1.52;
	return arm_moveit_joint->arm_joint1;
}

double Joint::arm_Joint1_moveit_2_real()
{
	arm_real_joint->arm_ptz = (-1520/3.32)*arm_moveit_joint->arm_joint1 - 695.9036;
	return arm_real_joint->arm_ptz;
}

double Joint::arm_Joint2_real_2_moveit()
{
	arm_moveit_joint->arm_joint2 = (-3.23/1430.0)*arm_real_joint->arm_joint2 + 1.43;
	return arm_moveit_joint->arm_joint2;
}


double Joint::arm_Joint2_moveit_2_real()
{
	arm_real_joint->arm_joint2 = (-1430/3.23)*arm_moveit_joint->arm_joint2 - 633.0960;
	return arm_real_joint->arm_joint2;
}

double Joint::arm_Pitch_real_2_moveit()
{
	arm_moveit_joint->arm_pitch = (1.0/100.0)*arm_real_joint->arm_pitch;
	return arm_moveit_joint->arm_pitch;
}
double Joint::arm_Pitch_moveit_2_real()
{
	arm_real_joint->arm_pitch = (100.0)*arm_moveit_joint->arm_pitch;
	return arm_real_joint->arm_pitch;
}

double Joint::arm_Roll_real_2_moveit()
{
	arm_moveit_joint->arm_roll = (3.14/180.0)*arm_real_joint->arm_roll;
	return arm_moveit_joint->arm_roll;
}
double Joint::arm_Roll_moveit_2_real()
{
	arm_real_joint->arm_roll = (180.0/3.14)*arm_moveit_joint->arm_roll;
	return arm_real_joint->arm_roll;
}

double Joint::arm_Gripper_real_2_moveit()
{
	return arm_moveit_joint->arm_gripper = 0.0;
}

double Joint::arm_Gripper_moveit_2_real()
{
	return arm_real_joint->arm_gripper = 0.0;
}


