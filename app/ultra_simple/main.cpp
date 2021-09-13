/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
//#include <signal.h>
#include <graphics.h>//for easy drawing
#include <algorithm> //for get maxelement
#include <vector> //for DBSCAN
#include <unordered_map>
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <windows.h> // for sound display
#include <stdbool.h>
#include <string.h>
#include <iostream>
#pragma comment(lib, "Winmm.lib") //for sound display

// #include <Winsock2.h>  
#pragma comment(lib,"ws2_32.lib")
#pragma warning(disable:4996)

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;
using namespace std;

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
    TimeOuts.ReadIntervalTimeout = 100; //读间隔超时 1000
    TimeOuts.ReadTotalTimeoutMultiplier = 100; //读时间系数 500
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

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}
using namespace std;
#define PI 3.1415926

typedef struct p {
    float first;
    float second;
}p;

//开全部变量做串口通信
WZSerialPort w;
//每次接受一个字符作为半径，故设置缓冲区
string buffer;
int radius_tmp = 5;
int get_turning_radius() {
    //预留接口做好准备
    if(buffer.empty())buffer = w.receive();//因为此时缓冲区取光了，这个时候再去取
    //这个时候讲道理应该是取到了，但要是没取到，那就会出错
    if (buffer.size()) {
        //radius_tmp = buffer[0] - '0';
        radius_tmp = atoi(buffer.substr(0,3).c_str());
        buffer = "";
    }
    int radius = radius_tmp << 1;//tmp:cm -> mm -> pix *10 /5 <=> (<<1)
    return radius;
}

WORD wVersionRequested;
WSADATA wsaData;
int err;
SOCKET sockClient;
SOCKADDR_IN addrSrv;
char* baseCh = { "1" };

void init_socket() {
    wVersionRequested = MAKEWORD(1, 1);
    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0) {
        exit(-1);
    }
    if (LOBYTE(wsaData.wVersion) != 1 ||
        HIBYTE(wsaData.wVersion) != 1) {
        WSACleanup();
        exit(-1);
    }
    sockClient = socket(AF_INET, SOCK_STREAM, 0);
    addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(8888);
    connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));

}

int get_turning_radius_by_socket() {
    send(sockClient, baseCh, strlen(baseCh) + 1, 0);
    char recvBuf[50];
    recv(sockClient, recvBuf, 50, 0);
    // printf("severRecv %s\n", recvBuf);
    radius_tmp = atoi(recvBuf);
    int radius = radius_tmp << 1;//tmp:cm -> mm -> pix *10 /5 <=> (<<1)
    return radius;
}

void close_socket() {
    closesocket(sockClient);
    WSACleanup();
}

void ExpandCluster(unordered_map<int,int> &mp,int visited[],
    vector<int> &inCluster,int &index,vector<vector<p>> &cluster,
    vector<int> &tmp, vector<p>& points,float metric,float MinPts,int useful) {

    vector<p> newCluster;
    newCluster.push_back(points[index]);//入口的点（核心点）放入新的簇中
    //printf("core point:%d\n", index);
    inCluster[index] = 1; //标记在簇中
    for (int i = 1; i < tmp.size();i++) { //领域可能会不断地扩大
        //i = 0 是肯定在里面的
        visited[tmp[i]] = 1;
        vector<int> neighbor;
        neighbor.push_back(tmp[i]);
        /*for (int j = 0; j < tmp.size(); j++) { //领域可能会不断地扩大
            if (tmp[i] == j)continue;
            if ((points[tmp[i]].first - points[tmp[j]].first)*(points[tmp[i]].first - points[tmp[j]].first)+
                (points[tmp[i]].second - points[tmp[j]].second)* (points[tmp[i]].second - points[tmp[j]].second)<=metric) {
                neighbor.push_back(tmp[j]);
            }
        }*/
        for (int j = 0; j < useful; j++) {
            if (tmp[i] == j)continue;
            if ((points[tmp[i]].first - points[j].first) * (points[tmp[i]].first - points[j].first) +
                (points[tmp[i]].second - points[j].second) * (points[tmp[i]].second - points[j].second) <= metric) {
                neighbor.push_back(j);
            }
        }
        if (neighbor.size() >= MinPts) {
            for (auto& n : neighbor) {
                if (!mp.count(n)) {
                    tmp.push_back(n);//扩展N的数目
                    mp[n] = 1;//标记进入领域中
                }
            }
        }
        if (!inCluster[tmp[i]]) {//如果不在任何簇中则加入新簇
            newCluster.push_back(points[tmp[i]]);
            inCluster[tmp[i]] = 1; //标记进入簇中
        }
    }
    cluster.push_back(newCluster);
}

/*
    int rbx = 320, rby = 240 + 260;
    以上是暂设置的后轮坐标
    半径预设是200
*/
int in_play;
inline bool indanger(float x, float y, float r,float turning_radius) {//半径接口
    if (abs(turning_radius) < 1e-7)return false;//直行确认没有危险区域
    int rbx = 320, rby = 240 + 300;//中心：320 240 安装位置到车身：1300mm/5 = 260pix 原有240+260
    int cxrb = rbx + turning_radius, cyrb = rby;//右后轮的转弯圆心坐标
    
    float dist2 = sqrt((x - cxrb) * (x - cxrb) + (y - cyrb) * (y - cyrb));
    if (dist2 >= turning_radius - r && y <= cyrb) { 
        return true; 
    }//保证其在第二象限中 半径接口
    return false;
}

int get_lspeed() {//轮速接口
    int speed = 50;
    return speed;//返回
}

int get_rspeed() {//轮速接口
    int speed = 50;
    return speed;//返回
}

void DBSCAN(int visited[],vector<p>& points,int useful,float Eps,float MinPts,int turning_radius) {
    int num = 0;//簇的个数
    int inner = 0; //簇内部点的个数 用于标记是不是噪声
    vector<int> isNoise(useful);//用于标记是否是噪声
    vector<vector<p>> Cluster;
    vector<int> inCluster(useful);
    unordered_map<int, int> mp; //用以标记是否在领域中

    float metric = Eps * Eps;
    for (int i = 0; i < useful;i++) {
        if (visited[i])continue;
        visited[i] = 1;
        //vector<p> tmp;
        vector<int> tmp;
        //tmp.push_back(points[i]);
        tmp.push_back(i);
        mp[i] = 1;
        for (int j = 0;j < useful;j++) {
            if (i == j)continue;
            //metric
            float distance = (points[i].first - points[j].first) * (points[i].first - points[j].first) +
                (points[i].second - points[j].second) * (points[i].second - points[j].second);
            if ( distance<=metric) {
                //tmp.push_back(points[j]);
                tmp.push_back(j);
                mp[j] = 1;
                //printf("%d ", distance);
            }
        }
        //num check
        if (tmp.size() < MinPts) {
            isNoise[i] = 1;
            continue;
        }//mark as noise
        //else
        ExpandCluster(mp,visited,inCluster,i,Cluster,tmp,points,metric,MinPts,useful);//inCluster index total_Cluster newCluster 
        //ATTENTION:metric as Eps^2
    }
    //计算聚类中心与半径
    setcolor(BLACK);
    int flag = 0;
    for (int i = 0;i < Cluster.size();i++) {
         float centerX = 0, centerY = 0;
         float left = 1e9, right = -1, down = -1, up = 1e9;
         
         for (int j = 0; j < Cluster[i].size();j++) {
             centerX += Cluster[i][j].first, centerY += Cluster[i][j].second;
             //printf("first:%lf second:%lf\n", Cluster[i][j].first, Cluster[i][j].second);
             left = min(left, Cluster[i][j].first), right = max(right,Cluster[i][j].first);
             up = min(up, Cluster[i][j].second), down = max(down, Cluster[i][j].second);
         }
         centerX /= Cluster[i].size(), centerY /= Cluster[i].size();
         //printf("centerX:%lf centerY:%lf\n", centerX,centerY);
         float radius = max(abs(centerX - left),max(abs(centerX - right),max(abs(centerY - up),abs(centerY - down))));
         radius = 10;
         //这里/5 使得真实距离mm能够转换到pix
         centerX = centerX /5 + 320, centerY = centerY /5 + 240;//加偏移量
         //printf("%d:horizon:%f vertical:%f radius:%f\n", i,centerX,centerY, radius);//修正了坐标之后应该没问题
         if (indanger(centerX, centerY, radius, turning_radius)) { 
             setfillcolor(RED); 
             flag = 1;
         }
         fillcircle((int)centerX, (int)centerY, (int)radius);
         setfillcolor(BLACK);
         //printf("%d:horizon:%f vertical:%f radius:%f\n", i,320 + centerX, 240 + centerY, radius);
    }
    if (flag) {
        if (!in_play)PlaySound("../static/tts_sample.wav", NULL, SND_FILENAME | SND_ASYNC | SND_LOOP);
        in_play = 1;
        //w.send("1");//1作为危险标志
    }
    else {
        //w.send("0");//0作为安全标志
        in_play = 0;
        PlaySound(NULL, NULL, SND_FILENAME | SND_ASYNC | SND_LOOP);
    }
}

void stm32_init() {
    DCB dcb;
    HANDLE m_hCom;//COM口句柄 
    m_hCom = CreateFile(
        "COM9",//预设COM9
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL);
    if (m_hCom != INVALID_HANDLE_VALUE) // 打开串行口成功 
    {
        // 配置通讯参数 
        GetCommState(m_hCom, &dcb);


        dcb.BaudRate = CBR_9600;//波特率 
        dcb.Parity = NOPARITY;
        //EVENPARITY 偶校验 ,NOPARITY 无校验 
        //MARKPARITY 标记校验   ODDPARITY 奇校验 
        dcb.ByteSize = 8;//数据位 
        dcb.StopBits = ONESTOPBIT;// ONESTOPBIT 1位停止位   
                                  //TWOSTOPBITS 2位停止位 
                                  //ONE5STOPBITS   1.5位停止位 

        COMMTIMEOUTS m_CommTimeouts;
        SetCommState(m_hCom, &dcb);
        // Config timeouts设置超时 
        m_CommTimeouts.ReadIntervalTimeout = 0;
        m_CommTimeouts.ReadTotalTimeoutConstant = 1000;
        m_CommTimeouts.ReadTotalTimeoutMultiplier = 0;
        m_CommTimeouts.WriteTotalTimeoutConstant = 200;
        m_CommTimeouts.WriteTotalTimeoutMultiplier = 0;
        SetCommTimeouts(m_hCom, &m_CommTimeouts);
        // Clear buffer清除缓冲区 
        PurgeComm(m_hCom, PURGE_TXABORT);
        PurgeComm(m_hCom, PURGE_RXABORT);
        PurgeComm(m_hCom, PURGE_TXCLEAR);
        PurgeComm(m_hCom, PURGE_RXCLEAR);

        // 打开串行口成功 
        //return TRUE;
    }
    
}

int main(int argc, const char * argv[]) {
    /*
    
    
    printf("请输入雷达串口:\n");
    int lidar_port = 0;
    scanf("%d", &lidar_port);

    string tmp_lidar_port = "\\\\.\\com" + (lidar_port + '0');
    cout << tmp_lidar_port << endl;
    opt_com_path = tmp_lidar_port.c_str();

    printf("%s\n", opt_com_path);
    */

    const char* opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    bool useArgcBaudrate = false;

    /*printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");*/

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        // opt_com_path = "\\\\.\\com3";  // 杨闻笛
        opt_com_path = "\\\\.\\com4"; // 刘欣怡
        // opt_com_path = "\\\\.\\com5"; // 辛亚行
#elif __APPLE__
        opt_com_path = "/dev/tty.SLAB_USBtoUART";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    
    

    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) 
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) 
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {
        
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    //signal(SIGINT, ctrlc);
    //try not to use signal
    
    {
        drv->startMotor();
        // start scan...
        drv->startScan(0, 1);

        // fetech result and print it out...
        // try to use GUI to demostrate data

        vector<p> tmp(8192, { 0,0 });
        init_socket();

        /*
            Area:Canvas setup
        */
        //距离调整
        bool flag = w.open("com4");
        /*if (!flag) {
            printf("cannot open com4");
            return 0;
        }*/
        initgraph(1280, 960);//修改画布大小之后是否应该重新修正centerX和centerY？
        int radius = 200; //radius of circle
        int centerX = 320, centerY = 240; //center of circle
        float maxDistance = 0.f; //Get maxDistance to resize canvas
        float minDistance = INT_MAX;
        float zoomFactor = 1.f;//set zoomFactor to fit into canvas
        setfillcolor(RED);//set fillcolor red to fill circle
        int fillRadius = 1;//set fill radius of dot cloud
        const int maxSize = 8192; //synchronized with 8192 (size of nodes array)
        const int tWidth = 100; 
        IMAGE img;
        setbkcolor(LIGHTGRAY);
        cleardevice();
        loadimage(&img,"../img/car_3.jpg",160,500);//实车修改 实车尺寸 长1.80m 宽（后轮轴距）0.75m
        //实车尺寸 长:2.50m 宽0.80m
        //2500/5 = 500 800/5 = 160
        in_play = 0;//声音标志初始化
        putimage(160, 80, &img);//显示位置标定 320 - 160 
        //纵坐标需要上移800/5 = 160 240-160 = 80
        setcolor(RED);//老板说改成红色
        settextcolor(BLACK);//设置文字颜色
        LOGFONT f;
        gettextstyle(&f);						// 获取当前字体设置
        f.lfHeight = 30;						// 设置字体高度为 48
        _tcscpy_s(f.lfFaceName, _T("黑体"));		// 设置字体为“黑体”(高版本 VC 推荐使用 _tcscpy_s 函数)
        f.lfQuality = ANTIALIASED_QUALITY;		// 设置输出效果为抗锯齿  
        settextstyle(&f);						// 设置字体样式
        
        int rbx = 320, rby = 240 + 300;//安装位置+安装位置到后轮长度 原240+260 现在多偏移一点
        int turning_radius = 0, turning_left_speed = 0, turning_right_speed = 0;

        /*
            轮速获取
        */
        //turning_radius = get_turning_radius();
        //turning_left_speed = get_lspeed();
        //turning_right_speed = get_rspeed();

        char r[10], ls[10], rs[10];
        if (turning_radius) {
            outtextxy(0, 0, _T("半径为:"));
            sprintf_s(r, "%d", turning_radius * 5);//转换成mm
            outtextxy(100, 0, r);
        }
        else {
            outtextxy(0, 0, _T("非右转"));
        }
        /*sprintf_s(ls, "%d", turning_left_speed);
        sprintf_s(rs, "%d", turning_right_speed);*/
        /*outtextxy(100,35, ls);
        outtextxy(100, 70, rs);*/
        if(turning_radius) arc(rbx, rby-turning_radius, rbx+(turning_radius<<1), rby+turning_radius, PI / 2, -PI);//半径接口
        //若转弯半径是0则不画
        /*
             eliminate 0 points
        */
        int useful = 0;
        while (1) {
            rplidar_response_measurement_node_t nodes[8192];//DEPRECATED API WARINING:grabScanData

            //rplidar_response_measurement_node_hq_t nodes[8192];
            size_t   count = _countof(nodes);

            op_result = drv->grabScanData(nodes, count); //DEPRECATED API WARINING
            //op_result = drv->grabScanDataHq(nodes, count);

            if (IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);//DEPRECATED API WARINING(Triggered by grabScanData)

                /*
                        Area:cout detail to screen
                */
                for (int pos = 0; pos < (int)count; ++pos) {

                    //IF TEST DATA ONLY
                    /*printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
                    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    nodes[pos].distance_q2/4.0f,
                    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);*/

                    /*
                        Area:Drawing
                    */

                    if ((nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) == 0)continue;
                    float tmpAngle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
                    if (tmpAngle >= 90 && tmpAngle <= 270)continue; //ONLY RIGHT SIDE WILL BE CHOOSEN
                    float tmpDistance = (nodes[pos].distance_q2 / 4.0f);
                    float tmpX = cos(tmpAngle * PI / 180) * tmpDistance;
                    float tmpY = sin(tmpAngle * PI / 180) * tmpDistance;
                    if (tmpY + 4800 < 0 || tmpY + 4800 > 9600 || tmpX + 6400 < 0 || tmpX + 6400 > 12800)continue; 
                    //若转换为画布上的坐标之后出界则不管
                    tmp[useful].first = tmpX;//first:x polar coordinate
                    tmp[useful].second = tmpY; //second:y polar coordinate
                    
                    /*
                        Area:min-max Distance metric
                    */
                    //maxDistance = max(maxDistance, tmpDistance);

                    //minDistance = tmpDistance < 1e-7 ? minDistance : min(minDistance, tmpDistance);// ~~~

                    //printf("dis:%03.2f theta: %03.2f X: %08.2f Y: %08.2f\n", tmpDistance,tmpAngle, tmp[useful].first, tmp[useful].second);
                    useful++;
                }
                //old
                /*
                    Area:maxDistance correction
                */
                //maxDistance = 1500; //~~~

                /*
                    Area:zoomFactor calcu
                */

                //zoomFactor = radius / maxDistance; //~~~
                /*
                    Area:Data visualation
                */
                /*for (int pos = 0;pos < (int)useful;++pos) {
                    solidcircle(tmp[pos].first,tmp[pos].second,fillRadius);//simple demo:radius=5
                }*/
                /*
                    Area:Text outtext
                */
                //circle(centerX, centerY, radius);
                //setcolor(GREEN);
                //line(320, 280, 350, 200);
                //printf(">>>useful:%d\n", useful);

                /*
                    Area:Cluster calculation
                */
                int visited[8192];
                memset(visited, 0, sizeof(visited));

                DBSCAN(visited, tmp, useful, 150, 10,turning_radius); //将转弯半径也传入 原：150 15
                Sleep(200);//延长时间？

                // 获取串口发来的数据
                turning_radius = get_turning_radius_by_socket();//通过接口刷新半径
                turning_left_speed = get_lspeed();
                turning_right_speed = get_rspeed();
                
                cleardevice();
                putimage(160, 80, &img);
                setcolor(BLACK);
                if (turning_radius) {
                    outtextxy(0, 0, _T("半径为:"));
                    sprintf_s(r, "%d", turning_radius * 5);//转换成mm
                    outtextxy(100, 0, r);
                }
                else {
                    outtextxy(0, 0, _T("非右转"));
                }
                /*outtextxy(0, 35, _T("左轮速:"));
                outtextxy(0, 70, _T("右轮速:"));*/
                /*sprintf_s(ls, "%d", turning_left_speed);
                sprintf_s(rs, "%d", turning_right_speed);*/
                /*outtextxy(100, 35, ls);
                outtextxy(100, 70, rs);*/
                setcolor(RED);
                if (turning_radius != 0) arc(rbx, rby - turning_radius, rbx + (turning_radius << 1), rby + turning_radius, PI / 2, -PI);//半径接口

                useful = 0;//trace 
            }

            /*if (ctrl_c_pressed){
                break;
            }*/
            //try not to use interrupt
            if (_kbhit()) {
                char sig = _getch();
                if (sig == 'q' || sig == 'Q') {
                    break;
                }
            }
        }
    }
    close_socket();
    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}
