/*
 * Command.h
 *
 */

#ifndef COMMAND_H_
#define COMMAND_H_

#include <string>
#include "Connect.h"

using namespace std;

//-------------------------   命令 类  ---------------
class Command
{
public:
	Command();
	virtual ~Command();

	//------Register Command-------------
	void RegisterCommand(string);

	//-----direct current drive------
	void DriveCurrent(int,int,bool);

	//-----direct velocity drive----
	void DriveVelocity(string,string,bool);  //--------速度驱动--------

	//------Go Command-------------
	void GoAhead(string,string,bool,bool);

	//------Rotate Command---------
	void Rotate(string,string,bool,bool);

	//------Flipper Current Command--------
	void FlipperCurrentCommand(string,string,string,string,bool);

	//------Flipper Velocity Command---------
	void FlipperVelocityCommand(string,string,string,string,bool);

	//------Flipper Position Command--------
	void FlipperPositionCommand(string,string,string,string,bool);

	//------Flipper Home Command-------------
	void FlipperHomeCommand(bool,bool,bool,bool);

	//------Arm Set Command-----------------
	void ArmSetCommand(string,string,string,string,string,string,string);

	//------Arm Home Command----------------
	void ArmHomeCommand(bool,bool,bool,bool,bool,bool,bool);

	//------Camera Change----------------
	void ChangeCamera(string,bool);

	//-------Arm X Z Axis Move Command------
	void ArmXZMoveCommand(string,string,string,string,string,bool);

	//--------       打开或者关闭电灯  ---------------------
	void lightStateCommand(string,bool);

	//----------
	void SetCommConnSock(int sock)
	{
		CONN.SetConnSock(sock);
	}

	string timeREGStr;

private:

	Connection CONN;

};

#endif /* COMMAND_H_ */
