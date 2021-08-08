#pragma once

#ifndef _WZSERIALPORT_H
#define _WZSERIALPORT_H
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <windows.h>
using namespace std;
/*
作者：欧阳伟
日期：2017-12-14
类名：WZSerialPort
用途：串口读写
示例：
WZSerialPort w;
if(w.open("COM1",9600，0，8，1))
{
w.send("helloworld",10);
char buf[1024];
w.receive(buf,1024);
}
*/

class WZSerialPort
{
public:
	WZSerialPort();
	~WZSerialPort();

	// 打开串口,成功返回true，失败返回false
	// portname(串口名): 在Windows下是"COM1""COM2"等，在Linux下是"/dev/ttyS1"等
	// baudrate(波特率): 9600、19200、38400、43000、56000、57600、115200 
	// parity(校验位): 0为无校验，1为奇校验，2为偶校验，3为标记校验
	// databit(数据位): 4-8，通常为8位
	// stopbit(停止位): 1为1位停止位，2为2位停止位,3为1.5位停止位
	// synchronizable(同步、异步): 0为异步，1为同步
	bool open(const char* portname, int baudrate = 9600, char parity = 0, char databit = 8, char stopbit = 1, char synchronizeflag = 0);

	//关闭串口，参数待定
	void close();

	//发送数据或写数据，成功返回发送数据长度，失败返回0
	int send(string dat);

	//接受数据或读数据，成功返回读取实际数据的长度，失败返回0
	string receive();

private:
	int pHandle[16];
	char synchronizeflag;
};

WZSerialPort::WZSerialPort()
{

}

WZSerialPort::~WZSerialPort()
{

}

bool WZSerialPort::open(const char* portname,
	int baudrate,
	char parity,
	char databit,
	char stopbit,
	char synchronizeflag)
{
	this->synchronizeflag = synchronizeflag;
	HANDLE hCom = NULL;
	if (this->synchronizeflag)
	{
		//同步方式
		hCom = CreateFileA(portname, //串口名
			GENERIC_READ | GENERIC_WRITE, //支持读写
			0, //独占方式，串口不支持共享
			NULL,//安全属性指针，默认值为NULL
			OPEN_EXISTING, //打开现有的串口文件
			0, //0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
			NULL);//用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL
	}
	else
	{
		//异步方式
		hCom = CreateFileA(portname, //串口名
			GENERIC_READ | GENERIC_WRITE, //支持读写
			0,			//独占方式，串口不支持共享
			NULL,	//安全属性指针，默认值为NULL
			OPEN_EXISTING, //打开现有的串口文件
			FILE_FLAG_OVERLAPPED, //0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
			NULL);//用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL
	}

	if (hCom == (HANDLE)-1)
	{
		return false;
	}

	//配置缓冲区大小 
	if (!SetupComm(hCom, 1024, 1024))
	{
		return false;
	}

	// 配置参数 
	DCB p;
	memset(&p, 0, sizeof(p));
	p.DCBlength = sizeof(p);
	p.BaudRate = baudrate; // 波特率
	p.ByteSize = databit; // 数据位

	switch (parity) //校验位
	{
	case 0:
		p.Parity = NOPARITY; //无校验
		break;
	case 1:
		p.Parity = ODDPARITY; //奇校验
		break;
	case 2:
		p.Parity = EVENPARITY; //偶校验
		break;
	case 3:
		p.Parity = MARKPARITY; //标记校验
		break;
	}

	switch (stopbit) //停止位
	{
	case 1:
		p.StopBits = ONESTOPBIT; //1位停止位
		break;
	case 2:
		p.StopBits = TWOSTOPBITS; //2位停止位
		break;
	case 3:
		p.StopBits = ONE5STOPBITS; //1.5位停止位
		break;
	}

	if (!SetCommState(hCom, &p))
	{
		// 设置参数失败
		return false;
	}

	//超时处理,单位：毫秒
	//总超时＝时间系数×读或写的字符数＋时间常量
	COMMTIMEOUTS TimeOuts;
	TimeOuts.ReadIntervalTimeout = 1000; //读间隔超时
	TimeOuts.ReadTotalTimeoutMultiplier = 500; //读时间系数
	TimeOuts.ReadTotalTimeoutConstant = 5000; //读时间常量
	TimeOuts.WriteTotalTimeoutMultiplier = 500; // 写时间系数
	TimeOuts.WriteTotalTimeoutConstant = 2000; //写时间常量
	SetCommTimeouts(hCom, &TimeOuts);

	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//清空串口缓冲区

	memcpy(pHandle, &hCom, sizeof(hCom));//保存句柄

	return true;
}

void WZSerialPort::close()
{
	HANDLE hCom = *(HANDLE*)pHandle;
	CloseHandle(hCom);
}

int WZSerialPort::send(string dat)
{
	HANDLE hCom = *(HANDLE*)pHandle;

	if (this->synchronizeflag)
	{
		// 同步方式
		DWORD dwBytesWrite = dat.length(); //成功写入的数据字节数
		BOOL bWriteStat = WriteFile(hCom, //串口句柄
			(char*)dat.c_str(), //数据首地址
			dwBytesWrite, //要发送的数据字节数
			&dwBytesWrite, //DWORD*，用来接收返回成功发送的数据字节数
			NULL); //NULL为同步发送，OVERLAPPED*为异步发送
		if (!bWriteStat)
		{
			return 0;
		}
		return dwBytesWrite;
	}
	else
	{
		//异步方式
		DWORD dwBytesWrite = dat.length(); //成功写入的数据字节数
		DWORD dwErrorFlags; //错误标志
		COMSTAT comStat; //通讯状态
		OVERLAPPED m_osWrite; //异步输入输出结构体

							  //创建一个用于OVERLAPPED的事件处理，不会真正用到，但系统要求这么做
		memset(&m_osWrite, 0, sizeof(m_osWrite));
		m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, "WriteEvent");

		ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误，获得设备当前状态
		BOOL bWriteStat = WriteFile(hCom, //串口句柄
			(char*)dat.c_str(), //数据首地址
			dwBytesWrite, //要发送的数据字节数
			&dwBytesWrite, //DWORD*，用来接收返回成功发送的数据字节数
			&m_osWrite); //NULL为同步发送，OVERLAPPED*为异步发送
		if (!bWriteStat)
		{
			if (GetLastError() == ERROR_IO_PENDING) //如果串口正在写入
			{
				WaitForSingleObject(m_osWrite.hEvent, 1000); //等待写入事件1秒钟
			}
			else
			{
				ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误
				CloseHandle(m_osWrite.hEvent); //关闭并释放hEvent内存
				return 0;
			}
		}
		return dwBytesWrite;
	}
}

string WZSerialPort::receive()
{
	HANDLE hCom = *(HANDLE*)pHandle;
	string rec_str = "";
	char buf[1024];
	if (this->synchronizeflag)
	{
		//同步方式
		DWORD wCount = 1024; //成功读取的数据字节数
		BOOL bReadStat = ReadFile(hCom, //串口句柄
			buf, //数据首地址
			wCount, //要读取的数据最大字节数
			&wCount, //DWORD*,用来接收返回成功读取的数据字节数
			NULL); //NULL为同步发送，OVERLAPPED*为异步发送
		for (int i = 0; i < 1024; i++)
		{
			if (buf[i] != -52)
				rec_str += buf[i];
			else
				break;
		}
		return rec_str;
	}
	else
	{
		//异步方式
		DWORD wCount = 1024; //成功读取的数据字节数
		DWORD dwErrorFlags;  //错误标志
		COMSTAT comStat;     //通讯状态
		OVERLAPPED m_osRead; //异步输入输出结构体

							 //创建一个用于OVERLAPPED的事件处理，不会真正用到，但系统要求这么做
		memset(&m_osRead, 0, sizeof(m_osRead));
		m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, "ReadEvent");

		ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误，获得设备当前状态
		//if (!comStat.cbInQue)    
		//	return 0;  //如果输入缓冲区字节数为0，则返回false

					   //std::cout << comStat.cbInQue << std::endl;
		BOOL bReadStat = ReadFile(hCom,     //串口句柄
			buf, //数据首地址
			wCount, //要读取的数据最大字节数
			&wCount, //DWORD*,用来接收返回成功读取的数据字节数
			&m_osRead); //NULL为同步发送，OVERLAPPED*为异步发送
		if (!bReadStat)
		{
			if (GetLastError() == ERROR_IO_PENDING) //如果串口正在读取中
			{
				//GetOverlappedResult函数的最后一个参数设为TRUE
				//函数会一直等待，直到读操作完成或由于错误而返回
				GetOverlappedResult(hCom, &m_osRead, &wCount, TRUE);
			}
			else
			{
				ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误
				CloseHandle(m_osRead.hEvent); //关闭并释放hEvent的内存
				return 0;
			}
		}
		for (int i = 0; i < 1024; i++)
		{
			if (buf[i] != -52)
				rec_str += buf[i];
			else
				break;
		}
		return rec_str;
	}
}
#endif