
#include "Command.h"


//Connection COMM;

Command::Command()
{
	
}

Command::~Command()
{
	
}

void Command::RegisterCommand(string timeREGStr)
{
	//timeREGStr=m_string_Time;
	string msg="REG {Mode WR} {Period ";
	msg+=timeREGStr;
	msg+="}\r\n";
	cout<<msg<<endl;
	CONN.sendMsg(msg);
}

void Command::DriveCurrent(int l,int r,bool n)
{
	string msg="DRIVE {Type Direct} {Name Current} {Left ";
	msg+=l;
	msg+="} {Right ";
	msg+=r;
	msg+="} {Normalized ";
	msg+=n ? "1}\r\n" : "0}\r\n";
	cout<<msg<<endl;
	CONN.sendMsg(msg);
}

void Command::DriveVelocity(string l,string r,bool n)
{
	string msg = "DRIVE {Type Direct} {Name Velocity} {Left ";
	msg+=l;
	msg+="} {Right ";
	msg+=r;
	msg+="} {Normalized ";
	msg+=n ? "1}\r\n" : "0}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}

void Command::GoAhead(string d,string v,bool n,bool p)
{
	string msg="GO {Type Ahead} {Distance ";
	msg+=d;
	msg+="} {Velocity ";
	msg+=v;
	msg+="} {Normalized ";
	msg+=n ? "1" : "0";
	msg+="} {Push ";
	msg+=p ? "1}\r\n" : "0}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}

void Command::Rotate(string a,string y,bool n,bool p)
{
	string msg = "GO {Type Rotate} {Angle ";
	msg += a;
	msg += "} {YawRate ";
	msg += y;
	msg += "} {Normalized ";
	msg += n ? "1" : "0";
	msg += "} {Push ";
	msg += p ? "1}\r\n" : "0}\r\n";
	CONN.sendMsg(msg);
}

void Command::FlipperCurrentCommand(string lf,string rf,string lb,string rb,bool n)
{
	string msg = "MULTIDRIVER {Type Direct} {Name Current} {LeftFront ";
	msg += lf;
	msg += "} {RightFront ";
	msg += rf;
	msg += "} {LeftBack ";
	msg += lb;
	msg += "} {RightBack ";
	msg += rb;
	msg += "} {Normalized ";
	msg += n ? "1}\r\n" : "0}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}

void Command::FlipperVelocityCommand(string lf,string rf,string lb,string rb,bool n)
{
	string msg = "MULTIDRIVER {Type Direct} {Name Velocity} {LeftFront ";
	msg += lf;
	msg += "} {RightFront ";
	msg += rf;
	msg += "} {LeftBack ";
	msg += lb;
	msg += "} {RightBack ";
	msg += rb;
	msg += "} {Normalized ";
	msg += n ? "1}\r\n" : "0}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}

void Command::FlipperPositionCommand(string lf,string rf,string lb,string rb,bool n)  //摆臂位置控制命令   RES是特殊情况
{
	string msg = "MULTISET {Type Direct} {Name Position} {LeftFront ";
	msg += lf;
	msg += "} {RightFront ";
	msg += rf;
	msg += "} {LeftBack ";
	msg += lb;
	msg += "} {RightBack ";
	msg += rb;
	msg += "} {Push ";
	msg += n ? "1}\r\n" : "0}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}

void Command::FlipperHomeCommand(bool lf,bool rf,bool lb,bool rb)
{
	string msg = "MULTISET {Type Direct} {Name Home} {LeftFront ";
	msg += lf ? "1" : "0";
	msg += "} {RightFront ";
	msg += rf ? "1" : "0";
	msg += "} {LeftBack ";
	msg += lb ? "1" : "0";
	msg += "} {RightBack ";
	msg += rb ? "1" : "0";
	msg += "}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}

void Command::ArmSetCommand(string PTZ,string j1,string j2,string screw,string p,string r,string g)
{
	string msg = "ARMSETV3 {Type Direct} {Name Position} {PTZ ";
	msg += PTZ;
	msg += "} {Joint1 ";
	msg += j1;
	msg += "} {Joint2 ";
	msg += j2;
	msg += "} {Stretch ";
	msg += screw;
	msg += "} {Pitch ";
	msg += p;
	msg += "} {Roll ";
	msg += r;
	msg += "} {Gripper ";
	msg += p ;
	msg+="}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}

void Command::lightStateCommand(string b,bool n)
{
	string msg = "SET {Type Light} {Bright ";
	msg += b;
	msg += "} {Normalized ";
	msg += n ? "1" : "0";
	msg += "}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}

void Command::ArmHomeCommand(bool PTZ,bool j1,bool j2,bool screw,bool p,bool r,bool g)
{
	string msg = "ARMSETV3 {Type Direct} {Name Home} {PTZ ";
	msg += PTZ ? "1" : "0";
	msg += "} {Joint1 ";
	msg += j1 ? "1" : "0";
	msg += "} {Joint2 ";
	msg += j2 ? "1" : "0";
	msg += "} {Stretch ";
	msg += screw ? "1" : "0";
	msg += "} {Pitch ";
	msg += p ? "1" : "0";
	msg += "} {Roll ";
	msg += r ? "1" : "0";
	msg += "} {Gripper ";
	msg += g ? "1" : "0";
	msg += "}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}
//----------------New Add Arm X Z Axis Move Command----------------
void Command::ArmXZMoveCommand(string x, string z, string P, string Y,string R,bool g)
{
	string msg = "ARMSETV3 {Type Local} {Name SensorPosition} {X ";
	msg += x;
	msg += "} {Y 0} {Z ";
	msg += z;
	msg += "} {Pitch ";
	msg += P;
	msg += "} {Yaw ";
	msg += Y;
	msg += "} {Roll ";
	msg += R;
	msg += "} {Push ";
	msg += g ? "1" : "0";
	msg += "}\r\n";
	CONN.sendMsg(msg);
	cout<<msg<<endl;
}
