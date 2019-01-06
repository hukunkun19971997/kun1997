#include"shixian.h"
#include "ZigBee.h"
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include "camera.h"    //

int source=1;
pthread_mutex_t mutex;
CAM jpeg;
int HexToArr(unsigned char * hexarr, unsigned char *arry, int silen);
void control_self(char *buf,char *curr,int fd,unsigned char *zigbee);
int  main()
{	

    //创建进程	
	pid_t pid;
    pid=fork();	
	if(pid==-1)
	{
		printf("error in fork\n");
		return -1;
	}
    //子进程采集发送照片
	else if(pid==0)
	{ 
       //线程 采集照片
       int ret;	
       pthread_t camera_tid;
	    //开启camera 线程			
	   ret = pthread_create(&camera_tid, NULL, thread_camera,NULL);
	   if (ret)
	   {		
		   perror("create camera thread");
		   exit(EXIT_FAILURE);
	   } 	
	   else
	   {
		   printf("create camera thread success\n");
	   }
	   selectcamer();
	   //thread_server();
       printf("+++++++++++\n");	
	}
//*******************************************************************************************   	
	//父进程	
	else
	{	
	int fdSerial=0;
	int iSetOpt = 0;//SetOpt 的增量i
    //配置串口
    if ((fdSerial = OpenPort(fdSerial, 4))<0)
	{
		perror("open_port error");
		//return -1;
	}
	//SetOpt(fdSerial, 115200, 8, 'N', 1)
	if ((iSetOpt = SetOpt(fdSerial, 115200, 8, 'N', 1))<0)
	{
		perror("set_opt error");
		//return -1;
	}
	printf("Serial fdSerial=%d\n", fdSerial);
	int recv_len = ReadDataTty(fdSerial, recv_zigbee_buf, 2, 36);
	if(recv_len<0)
	{
	    printf("no data\n");
	    //return -1;
	}
	
    int res;
    char *sendbuf = "hello client";
    char buf[50]="";
    struct sockaddr_in svr_addr, c_addr;
    int s_fd;//监听描述符
    int c_fd[99];//通信用描述符
    int count;//客户端数目计数器
    int s_len = sizeof(svr_addr);
    int c_len = sizeof(c_addr);
    fd_set fds;//被select监控的文件描述符的集合
    int maxfd;
    int i;
	
	char bufcontrol[50]="";
	
	char fourbuf[5]="";

    bzero(&svr_addr, s_len);
    bzero(&c_addr, c_len);
	char send_client[65] = "";
	char buff2[10] = "";
    //建立套接字
    s_fd = socket(AF_INET, SOCK_STREAM, 0);

    //设置套接字对应的地址 端口号
    svr_addr.sin_family = AF_INET;//设定地址协议族
    svr_addr.sin_port = htons(6666);//设定端口号
    inet_pton(AF_INET,"192.168.0.145",&svr_addr.sin_addr);//转换地址

    //绑定
    res = bind(s_fd, (struct sockaddr *)&svr_addr, s_len);
    if(res != 0)
    {
		perror("bind————————————————+++ ");
        return ;
    }
    //监听
    listen(s_fd,5);
    //初始化
    count =0;
    maxfd=0;
    FD_ZERO(&fds);
    for(i=0;i<99;i++)
    {
        c_fd[i]=-1;
    }
    while(1)
    {	
        FD_ZERO(&fds);
        maxfd=0;
        //构造文件描述符集合
        //1 加入监听文件描述符
        FD_SET(s_fd, &fds);
        maxfd = maxfd >= s_fd ? maxfd : s_fd;
        //2 加入通信文件描述符
        for(i=0;i<count;i++)
        {
            if(c_fd[i] != -1)
            {
                FD_SET(c_fd[i],&fds);
                maxfd = maxfd >= c_fd[i] ? maxfd : c_fd[i];
            }
        }

        //使用select循环控制描述符集合
          res = select(maxfd+1, &fds ,NULL,NULL,NULL);

        //分情况处理
        //1 有客户端连接，监听文件描述符改变
       if(FD_ISSET(s_fd, &fds))
       {
           c_fd[count] = accept(s_fd, (struct sockaddr *)&c_addr, &c_len);
           if(c_fd[count]>0)
		   {           
               count++;
           }
       }
        //2 有客户发送数据， 通信文件描述符改变
       for(i=0;i<count;i++)
       {
           //判定被改变的文件描述符是否在集合中
           if(c_fd[i] != -1 && FD_ISSET(c_fd[i], &fds))
           {
               //读取数据
               res = recv(c_fd[i],buf,50,0);

               if(res == 0)
               {
                   printf("client quit\n");
                   close(c_fd[i]);
                   c_fd[i]=-1;
               }

               if(res == -1)
               {
                   printf("something not going well\n");
                   close(c_fd[i]);
                   c_fd[i]=-1;
               }
			   
			   //io 操作
               if(res >0)
               {
                    printf("%s\r\n",buf);
					strncpy(fourbuf,buf,4);     //获取命令码
					if(strcmp(fourbuf,"gggg")==0)
					{
						strcpy(buf,"pppp");
						send(c_fd[i],buf,strlen(buf),0);
						memset(buf, 0, sizeof(buf));
						memset(fourbuf, 0, sizeof(fourbuf));
					}
					else if(strcmp(fourbuf,"lgin")==0)
					{
						//判断 用户名 密码 是否正确
						int Ret= Client_load( buf);//
						if(0==Ret)
						{
							strcpy(buf,"lgok");
							send(c_fd[i],buf,strlen(buf),0);
							memset(buf, 0, sizeof(fourbuf));
						}
						else
						{
							strcpy(buf,"lger\r\n");
							send(c_fd[i],buf,strlen(buf),0);
							memset(buf, 0, sizeof(fourbuf));
						}
							memset(fourbuf, 0, sizeof(fourbuf));
					}
					else if (strcmp(fourbuf,"lgrd")==0)
					{
						printf("%s\r\n",fourbuf);
						strcpy(buf,"dalo");
						send(c_fd[i],buf,strlen(buf),0);
						memset(buf, 0, sizeof(buf));
						memset(fourbuf, 0, sizeof(fourbuf));
					}
					else if (strcmp(fourbuf,"dard")==0)   //发送数据报文
					{
						printf("%s\r\n",fourbuf);
				        printf("%s\n",send_client_buf);
						send(c_fd[i],send_client,sizeof(send_client),0);
						strcpy(bufcontrol,send_client);
						memset(buf, 0, sizeof(buf));
						memset(fourbuf, 0, sizeof(fourbuf));
					}
					else if (strcmp(fourbuf,"ctpy")==0)   //接受控制命令
					{
						//打印控制命令                         
						printf("控制: %s\n", buf);
						//pc客户端命令转 m0控制命令
						printf("---------------------------------\n");
						ClientToSwrver(fdSerial, buf, 32, send_zigbee_buf, set_env, 36);
						control_self(buf,send_client,fdSerial,send_zigbee_buf);
				        printf("+++++++++++++++++++++++++++++++++\n");
						strcpy(buf,"ctok");
						send(c_fd[i],buf,sizeof(buf),0);
						memset(buf, 0, sizeof(buf));
						memset(fourbuf, 0, sizeof(fourbuf));
					}
					//发送报文修改
					recv_len = ReadDataTty(fdSerial, recv_zigbee_buf, 2, 36);//读到不为空
					int i;
					if(0 < recv_len)
					{
						if(-1 != ServerToClient(recv_zigbee_buf, 36, send_client_buf, 32))
						{
							printf("\n");
							printf("Temp: %d.%d \n", (int)send_client_buf[5], (int)send_client_buf[6]);
							printf("Hum: %d.%d  \n", (int)send_client_buf[7], (int)send_client_buf[8]);
							printf("Light: %d\n",(int)send_client_buf[9]);
							printf("Voltage: %g V\n",(float)send_client_buf[10] / 10);
							printf("\n");
							strcpy(send_client, "dapy#");
							printf("send_client[] = %s\n", send_client);
							for(i = 5;i < 32; i++)
							{	
								if(9 == i)
								{
									sprintf(buff2, "%03d", (unsigned int)send_client_buf[i]);
									strcat(send_client, buff2);
									memset(buff2, 0, 10);
								}
								else
								{
									sprintf(buff2, "%02d", (unsigned int)send_client_buf[i]);
									strcat(send_client, buff2);
									memset(buff2, 0, 10);
								}
							} //for
							printf("send_client 1 = %s\n", send_client);
						}//if
					}//if
				}//if
			}//if
		}//for
	}//for 
   
	}
return 0;
}
/*********************************************************
* 函数名：OpenPort
* 函数功能：打开一个USB设备
* 函数入参：fd 文件描述符
*           comport 需打开的设备 
*				1 -- /dev/ttyS0
*               2 -- /dev/ttyS1
*				3 -- /dev/ttyS2
*				4 -- /dev/ttyUSB0
* 函数返回值：失败返回 -1， 成功返回打开的设备号 
*********************************************************/
int OpenPort(int fd, int comport)
{

	if (comport == 1)
	{
		fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd)
		{
			perror("Can't Open Serial Port");
			return(-1);
		}
		else
		{
			printf("open ttyS0 .....\n");
		}
	}
	else if (comport == 2)
	{
		fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd)
		{
			perror("Can't Open Serial Port");
			return(-1);
		}
		else
		{
			printf("open ttyS1 .....\n");
		}
	}
	else if (comport == 3)
	{
		fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd)
		{
			perror("Can't Open Serial Port");
			return(-1);
		}
		else
		{
			printf("open ttyS2 .....\n");
		}
	}
	else if (comport == 4)
	{
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd)
		{
			perror("Can't Open Serial Port");
			return(-1);
		}
		else
		{
			printf("open ttyUSB0 .....\n");
		}
	}	
	/*******************************************************************
	* fcntl()针对(文件)描述符提供控制。参数fd是被参数cmd操作(如下面的描述)
	* 的描述符。针对cmd的值，fcntl能够接受第三个参数int arg。
	* fcntl函数有5种功能：
	*	 1. 复制一个现有的描述符(cmd=F_DUPFD).
	*	 2. 获得／设置文件描述符标记(cmd=F_GETFD或F_SETFD).
	*	 3. 获得／设置文件状态标记(cmd=F_GETFL或F_SETFL).?
	*	 4. 获得／设置异步I/O所有权(cmd=F_GETOWN或F_SETOWN).
	*	 5. 获得／设置记录锁(cmd=F_GETLK , F_SETLK或F_SETLKW).
	* fcntl()的返回值与命令有关。如果出错，所有命令都返回－1，如果成功则
	* 返回某个其他值。下列三个命令有特定返回值：F_DUPFD , F_GETFD , F_GETFL
	* 以及F_GETOWN。
	*	 F_DUPFD  返回新的文件描述符
	*	 F_GETFD  返回相应标志
	*	 F_GETFL , F_GETOWN 返回一个正的进程ID或负的进程组ID
	* F_SETFL 设置给arg描述符状态标志，可以更改的几个标志是：O_APPEND，
	* O_NONBLOCK，O_SYNC 和 O_ASYNC。而fcntl的文件状态标志总共有7个：O_RDONLY,
	* O_WRONLY , O_RDWR , O_APPEND , O_NONBLOCK , O_SYNC和O_ASYNC
	**********************************************************************/
	if (fcntl(fd, F_SETFL, 0)<0)
	{
		printf("fcntl failed!\n");
	}
	else
	{
		printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
	}
	//isatty，函数名。主要功能是检查设备类型 ，判断文件描述词是否是为终端机。
	if (isatty(STDIN_FILENO) == 0)
	{
		printf("standard input is not a terminal device\n");
	}
	else
	{
		printf("is a tty success!\n");
	}
	printf("fd-open=%d\n", fd);
	return fd;
}
/*****************************************************************************
* 函数名：SetOpt
* 函数功能：配置串口
* 函数入参：fd 文件描述符
*			nSpeed 波特率（bps） 2400 4800 9600 115200 
*			nBits 数据位 
*			nEvent 校验位 O 奇校验  E 偶校验  N 无校验
*			nStop 停止位
* 函数返回值：成功返回 0，失败返回 -1.
****************************************************************************/
int SetOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio, oldtio;
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch (nBits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}

	switch (nEvent)
	{
	case 'O':                     //奇校验
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':                     //偶校验
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':                    //无校验
		newtio.c_cflag &= ~PARENB;
		break;
	}

	switch (nSpeed)
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}
	if (nStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (nStop == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}
	printf("set done!\n");
	return 0;
}

/*****************************************************************************
* 函数名：ReadDataTty
* 函数功能：读取一帧数据
* 函数入参：fd 文件描述符
*			rcv_buf 接收缓冲区
*			TimeOut 超时时间
*			len 接收字符长度
* 函数返回值：成功返回读到的字节，失败返回一个负数错误码。
****************************************************************************/

int ReadDataTty(int fd, unsigned char *rcv_buf, int TimeOut, int Len)
{
	if(NULL == rcv_buf)
	{
		return -1;
	}
	int retval;
	fd_set rfds;
	struct timeval tv;
	int ret, pos;
	tv.tv_sec = TimeOut / 1000;  //set the rcv wait time  
	tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s  

	pos = 0;
	while (1)
	{
		tv.tv_sec = TimeOut / 1000;  //set the rcv wait time  
		tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s  
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		retval = select(fd + 1, &rfds, NULL, NULL, &tv);
		if (retval == -1)
		{
			perror("select()");
			break;
		}
		else if (retval)
		{
			ret = read(fd, rcv_buf + pos, 1);
			if (-1 == ret)
			{
				break;
			}

			pos++;
			if (Len <= pos)
			{
				break;
			}
		}
		else
		{
			break;
		}
	}

	return pos;
}

/*****************************************************************************
* 函数名：SendDataTty
* 函数功能：发送一帧数据
* 函数入参：fd 文件描述符
*			send_buf 发送缓冲区
*			TimeOut 发送超时
*			len 发送字节长度
* 函数返回值：成功返回 发送字节数， 失败返回 负数错误码。
****************************************************************************/
int SendDataTty(int fd, unsigned char *send_buf, int TimeOut, int Len)
{
	int i=0;
	for(i=0;i<5;i++)
	{
	printf("控制命令 %x\n",send_buf[i]);
	}
	if(NULL == send_buf)
	{
		return -1;
	}
	/*
	struct timeval tv;
	int ret;
	tv.tv_sec = TimeOut / 1000;  //set the rcv wait time  
	tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s  

	tv.tv_sec = TimeOut / 1000;  //set the rcv wait time  
	tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s */ 
	int ret = write(fd, send_buf, Len);
	if (-1 == ret)
	{
		printf("write device error\n");
		return 0;
	}
	printf("成功控制返回\n");
	sleep(2);
	return ret;
}

/*****************************************************************************
* 函数名：IntelligentControl
* 函数功能：根据传感器采集数据自动控制。
* 函数入参：fd 文件描述符
*			rcv_buf 接受字节序
*           send_buf 控制字节序
*           TimeOut 超时时间
*           len 字节长度
* 函数返回值：成功返回 0，失败返回一个负数错误码。
****************************************************************************/
int IntelligentControl(int fd, unsigned char *rcv_buf, unsigned char *send_buf,\
						int TimeOut, int Len)
{
	if(NULL == rcv_buf && NULL == send_buf)
	{
		return -1;
	}
	send_buf[0] = (unsigned char)0xdd;
	send_buf[1] = (unsigned char)0x07;
	send_buf[2] = (unsigned char)0x24;
	send_buf[3] = (unsigned char)0x00;
	if((rcv_buf[5] - set_env[0]) > 0)
	{
		send_buf[4] = (unsigned char)0x00;
		SendDataTty(fd, send_buf, TimeOut, BUFFLEN);
	}
	else
	{
		send_buf[4] = (unsigned char)0x01;
		SendDataTty(fd, send_buf, TimeOut, BUFFLEN);
	}
	if((rcv_buf[7] - set_env[2]) > 0)
	{
		send_buf[4] = (unsigned char)0x02; //0x02 (开)  
		SendDataTty(fd, send_buf, TimeOut, BUFFLEN);
	}
	else
	{
		send_buf[4] = (unsigned char)0x03;
		SendDataTty(fd, send_buf, TimeOut, BUFFLEN);
	}
	if((rcv_buf[8] - set_env[4]) > 0)
	{
		send_buf[4] = (unsigned char)0x04;
		SendDataTty(fd, send_buf, TimeOut, BUFFLEN);
	}
	else
	{
		send_buf[4] = (unsigned char)0x08;
		SendDataTty(fd, send_buf, TimeOut, BUFFLEN);
	}
	if((rcv_buf[9] - set_env[5]) > 0)
	{
		send_buf[4] = (unsigned char)0x09;
		SendDataTty(fd, send_buf, TimeOut, BUFFLEN);
	}
	else
	{
		send_buf[4] = (unsigned char)0x0A;
		SendDataTty(fd, send_buf, TimeOut, BUFFLEN);
	}
	return 0;
}

//自动控制
void control_self(char *buf,char *curr,int fd,unsigned char *zigbee)
{
	zigbee[0] = (unsigned char)0xdd;
	zigbee[1] = (unsigned char)0x07;
	zigbee[2] = (unsigned char)0x24;
	zigbee[3] = (unsigned char)0x00;
	zigbee[4] = (unsigned char)0xcc;
	
	int currtem;
	int currhum;
	char currtems[3]={curr[5],curr[6],'\0'};
	char currhums[3]={curr[9],curr[10],'\0'};
	currtem=atoi(currtems);
	currhum=atoi(currhums);
	
	 	
	int tem;
	int hum;
	char tems[3]={buf[11],buf[12],'\0'};
	char hums[3]={buf[15],buf[16],'\0'};
	tem=atoi(tems);
	hum=atoi(hums);
    printf("当前温度 %d  控制温度 %d\n",currtem,tem);
	printf("当前湿度 %d  控制湿度 %d\n",currhum,hum);
	if(currtem > tem)
	{
		printf("打开空调、\n");
		zigbee[4] = (unsigned char)0x04;
		SendDataTty(fd, zigbee, TIMEOUT, BUFFLEN);
		if(currtem > 27)
		{
			sleep(2);
			zigbee[4] = (unsigned char)0x02;
		    SendDataTty(fd, zigbee, TIMEOUT, BUFFLEN);
		}
	}
	else 
	{
		if(50 > tem && currtem< tem)
		{
		printf("关闭空调、\n");
		zigbee[4] = (unsigned char)0x08;
		SendDataTty(fd, zigbee, TIMEOUT, BUFFLEN);
		}
	}
	if(currhum > hum)
	{
		printf("打开除湿机、\n");
		zigbee[4] = (unsigned char)0x09;
		SendDataTty(fd, zigbee, TIMEOUT, BUFFLEN);
	}
	else
	{
		if(80>hum&&hum > currhum)
		{
		printf("关闭除湿机、\n");
		zigbee[4] = (unsigned char)0x0A;
		SendDataTty(fd, zigbee, TIMEOUT, BUFFLEN);
		}
	}
}

/*****************************************************************************
* 函数名：ServerToClient
* 函数功能：ZigBee字节序转客户端字节序（ZigBee --> network）
* 函数入参：zigbee  ZigBee 通讯数据 
*           ziglen  ZigBee数据长度
*           network 网路通讯数据
*           netlen  网络通讯数据长度
* 函数返回值：成功返回 0 失败返回负数错误码
****************************************************************************/	
int ServerToClient(unsigned char *zigbee, int ziglen, unsigned char *network, int netlen)
{
	int err = -1;
	if(NULL == zigbee && NULL == network)
	{
		return -1;
	}
	if(0xbb != *zigbee)//不是数据命令退出
	{
		return -1;
	}
	if(0x07 != zigbee[1])
	{
		printf("Other ID: %d\n", (unsigned int)zigbee[1]);
		return -1;
	}
	printf("Add ID: %d\n", (unsigned int)zigbee[1]);
	strncpy(network, "dapy#", 5);
	
	*(network + 5) = *(zigbee + 5); //温度整数 
	network[6] = zigbee[4]; //温度小数
	network[7] = zigbee[7]; //湿度整数 
	network[8] = zigbee[6]; //湿度小数 
	network[9] = zigbee[20]; //光线传感器 
	network[10] = zigbee[12];//电位器 
	network[18] = '#';      //分隔符
	network[19] = zigbee[24]; //LED状态
	network[20] = zigbee[26];//门状态
	if(0x01 == zigbee[25])
	{
		network[21] = 'A';//风扇状态 开
	}
	else
	{
		network[21] = 'B'; //风扇状态 关
	}
	if(0x00 == zigbee[27])
	{
		network[22] = 'A';//加湿器状态 开 
	}
	else
	{
		network[22] = 'B';//解释器状态 关
	}
	network[23] = 'A';       //总开关状态
	return 0;
}


/*****************************************************************************
* 函数名：ClientToSwrver
* 函数功能：客户端字节序转ZigBee字节序（network --> ZigBee）并且控制硬件完成操作
* 函数入参：fd      文件描述符
* 			ziglen  ZigBee数据长度
*           network 网路通讯数据
*           zigbee  ZigBee 通讯数据 
*           ziglen  ZigBee数据长度
*           env     环境参数设置:
* 函数返回值：成功返回 0 失败返回负数错误码
****************************************************************************/
int ClientToSwrver(int fd, unsigned char *network, int netlen, unsigned char *zigbee,\
		unsigned char *env, int ziglen)
{
	if(NULL == network || NULL == zigbee || NULL == env)
	{
		return -1;
	}
	unsigned char on[5] = {0x00, 0x02, 0x04, 0x09};
	//开LED, 开蜂鸣器,开风扇,开数码管
	unsigned char off[5] = {0x01, 0x03, 0x08, 0x0A, 0x0B};
	zigbee[0] = (unsigned char)0xdd;
	zigbee[1] = (unsigned char)0x07;
	zigbee[2] = (unsigned char)0x24;
	zigbee[3] = (unsigned char)0x00;
	zigbee[4] = (unsigned char)0xcc;
	
	//关闭LED, 关蜂鸣器, 关风扇, 关数码管, 关机
	int i=0;                        //修改 0615 17：
	int j = 0;
	if(NULL == zigbee && NULL == network)
	{
		return -1;
	}
	if( NULL == strstr(network, "ctpy"))//命令码错误
	{
		printf("命令码错误\n");
		return -1;
	}
	int temp = 0; //温度 
	int hum = 0; //湿度
	printf("预设环境: %s\n", network);
	/*
	for(i = 11; i < 2; i++)
	{
		if('0' <= network[i] && '9' >= network[i])
		{
			printf("network[%d] = %c", i, network[i]);
			temp = ((temp * 10) + (network[i] - '0'));
		}
	}*/
	printf("预设温度: %d 摄氏度\n", temp);
	/*
	for(i = 15; i < 2; i++)
	{
	//	if('0' <= network[i] && '9' >= network[i])
	//	{
			printf("network[%d] = %c", i, network[i]);
			hum = ((hum * 10) + (network[i] - '0'));
	//	}
	}*/
	printf("预设湿度: %d \n", hum);
	for(i = 5; i< 10; i++)
	{
		if('A' == network[i] || 'a' == network[i])        //打开led
		{
			printf("%c\n",network[i]);
			zigbee[4] = on[j++];
			printf("控制命令%x\n",zigbee[4]);
			SendDataTty(fd, zigbee, TIMEOUT, BUFFLEN);
			return 0;
		}
		else if('B' == network[i] || 'b' == network[i])
		{
			printf("%c\n",network[i]);
			zigbee[4] = off[j++];
			SendDataTty(fd, zigbee, TIMEOUT, BUFFLEN);
			printf("控制命令%x\n",zigbee[4]);
			return 0;
		}
		else if('c' == network[i])
		{
		    j++;
		}	
	}
	printf("无控制字节序\n");//测试用可删
	return 0;
}

int HexToArr(unsigned char * hexarr, unsigned char *arry, int silen)
{
	if(NULL == hexarr || NULL == arry)
	{
		return -1;
	}
	int i;
	int len = 0;
	unsigned char temp;
	for(i = 0; i < silen; i++)
	{
		temp = hexarr[i] >> 4;
		if(temp >= 0x00 && temp <= 0x09)
		{
			arry[len++] = temp + '0';
		}
		else if(temp >= 0x0a && temp <= 0x0f)
		{
			arry[len++] = temp + (unsigned char)55;
		}
		temp = hexarr[i] << 4;
		temp = temp >> 4;
		if(temp >= 0x00 && temp <= 0x09)
		{
			arry[len++] = temp + '0';
		}
		else if(temp >= 0x0a && temp <= 0x0f)
		{
			arry[len++] = temp + (unsigned char)55;
		}
	}
	arry[len] = '\0';
	return len;
}
