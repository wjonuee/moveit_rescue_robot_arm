#include "Connect.h"

Connection::Connection()
{
	conn_sock=-1;
	cout<<"Connection"<<conn_sock<<endl;
	buffer=new char[buffer_size];
	rcvbuffer=new char[buffer_size];
	recv_size=0;
	offset=0;

	time_t t = time(0);
	char temp[60];
	strftime(temp, sizeof(temp), "%Y_%m_%d %H:%M:%S", localtime(&t));
	arm_feedback_data_filename = temp;
	arm_feedback_data_filename = "/home/wjonuee/desktop/"+arm_feedback_data_filename+".txt";
	cout<<arm_feedback_data_filename<<endl;
	ofile1.open(arm_feedback_data_filename.c_str());

}

Connection::~Connection()
{
	delete[]buffer;
	delete[]rcvbuffer;
}

//------------------ 初始化 建立socket连接  与机器人建立连接  ----------------
bool Connection::Oninit(string servip,int servport)             
{
	conn_sock = socket(AF_INET,SOCK_STREAM,0);              //---------创建套接字---------

	if (conn_sock == -1)
	{
		return false;
	}
	else
	{
		cout<<"socket is success"<<endl;
	}

	bzero(&serv_addr,sizeof(serv_addr));
	serv_addr.sin_family=AF_INET;
	serv_addr.sin_port=htons(servport);
	serv_addr.sin_addr.s_addr=inet_addr(servip.c_str());

	if (connect(conn_sock,(struct sockaddr*)&serv_addr,sizeof(struct sockaddr))==-1)  //------与机器人连接 --------
	{
		return false;
	}
	else
	{
		cout<<"Connect is successful!"<<endl;
	}

	return true;

}

//-----------------------------------  发送消息 ------------------
bool Connection::sendMsg(string msg)       
{
	cout<<msg<<endl;
	int sendbytes=msg.length();

	if (send(conn_sock,msg.c_str(),sendbytes,0) != sendbytes)
	{
		perror("send command");
		cout<<"sendMsg"<<conn_sock<<endl;
		//exit(1);
		return false;
	}

	return true;
}

//-------------------------------------  接受消息包----------------
bool Connection::recvMsg()
{
	ofstream ofile;
	ofile.open("/home/wjonuee/desktop/log.txt");//,ios::app);
	bzero(rcvbuffer,buffer_size);
	if ((recv_size = recv(conn_sock, rcvbuffer, buffer_size, 0)) == -1)
	{
		perror("receive data");
		//exit(1);
		return false;
	}

	memmove(buffer+offset,rcvbuffer,recv_size);

	for (int i=0;i<recv_size;i++)
	{
		if (rcvbuffer[i]==0)
		{
			rcvbuffer[i]=0x20;
		}

		//ofile<<rcvbuffer[i];
	}

	string stemp(rcvbuffer);
	ofile1<<stemp<<endl<<endl<<endl;
	ofile<<stemp<<endl;
	ofile.close();

	return true;
}

void Connection::deleteBuff()
{
	ofile1.close();	
	delete[]rcvbuffer;
	delete[]buffer;
}

void Connection::closeConnection()
{
	close(conn_sock);
	cout<<"Close the communication with the robot"<<endl;
	offset=0;
}
