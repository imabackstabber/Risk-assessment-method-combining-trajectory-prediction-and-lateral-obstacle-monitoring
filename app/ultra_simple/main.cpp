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
#pragma comment(lib, "Winmm.lib") //for sound display

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

    // �򿪴���,�ɹ�����true��ʧ�ܷ���false
    // portname(������): ��Windows����"COM1""COM2"�ȣ���Linux����"/dev/ttyS1"��
    // baudrate(������): 9600��19200��38400��43000��56000��57600��115200 
    // parity(У��λ): 0Ϊ��У�飬1Ϊ��У�飬2ΪżУ�飬3Ϊ���У��
    // databit(����λ): 4-8��ͨ��Ϊ8λ
    // stopbit(ֹͣλ): 1Ϊ1λֹͣλ��2Ϊ2λֹͣλ,3Ϊ1.5λֹͣλ
    // synchronizable(ͬ�����첽): 0Ϊ�첽��1Ϊͬ��
    bool open(const char* portname, int baudrate = 9600, char parity = 0, char databit = 8, char stopbit = 1, char synchronizeflag = 0);

    //�رմ��ڣ���������
    void close();

    //�������ݻ�д���ݣ��ɹ����ط������ݳ��ȣ�ʧ�ܷ���0
    int send(string dat);

    //�������ݻ�����ݣ��ɹ����ض�ȡʵ�����ݵĳ��ȣ�ʧ�ܷ���0
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
        //ͬ����ʽ
        hCom = CreateFileA(portname, //������
            GENERIC_READ | GENERIC_WRITE, //֧�ֶ�д
            0, //��ռ��ʽ�����ڲ�֧�ֹ���
            NULL,//��ȫ����ָ�룬Ĭ��ֵΪNULL
            OPEN_EXISTING, //�����еĴ����ļ�
            0, //0��ͬ����ʽ��FILE_FLAG_OVERLAPPED���첽��ʽ
            NULL);//���ڸ����ļ������Ĭ��ֵΪNULL���Դ��ڶ��Ըò���������ΪNULL
    }
    else
    {
        //�첽��ʽ
        hCom = CreateFileA(portname, //������
            GENERIC_READ | GENERIC_WRITE, //֧�ֶ�д
            0,			//��ռ��ʽ�����ڲ�֧�ֹ���
            NULL,	//��ȫ����ָ�룬Ĭ��ֵΪNULL
            OPEN_EXISTING, //�����еĴ����ļ�
            FILE_FLAG_OVERLAPPED, //0��ͬ����ʽ��FILE_FLAG_OVERLAPPED���첽��ʽ
            NULL);//���ڸ����ļ������Ĭ��ֵΪNULL���Դ��ڶ��Ըò���������ΪNULL
    }

    if (hCom == (HANDLE)-1)
    {
        return false;
    }

    //���û�������С 
    if (!SetupComm(hCom, 1024, 1024))
    {
        return false;
    }

    // ���ò��� 
    DCB p;
    memset(&p, 0, sizeof(p));
    p.DCBlength = sizeof(p);
    p.BaudRate = baudrate; // ������
    p.ByteSize = databit; // ����λ

    switch (parity) //У��λ
    {
    case 0:
        p.Parity = NOPARITY; //��У��
        break;
    case 1:
        p.Parity = ODDPARITY; //��У��
        break;
    case 2:
        p.Parity = EVENPARITY; //żУ��
        break;
    case 3:
        p.Parity = MARKPARITY; //���У��
        break;
    }

    switch (stopbit) //ֹͣλ
    {
    case 1:
        p.StopBits = ONESTOPBIT; //1λֹͣλ
        break;
    case 2:
        p.StopBits = TWOSTOPBITS; //2λֹͣλ
        break;
    case 3:
        p.StopBits = ONE5STOPBITS; //1.5λֹͣλ
        break;
    }

    if (!SetCommState(hCom, &p))
    {
        // ���ò���ʧ��
        return false;
    }

    //��ʱ����,��λ������
    //�ܳ�ʱ��ʱ��ϵ��������д���ַ�����ʱ�䳣��
    COMMTIMEOUTS TimeOuts;
    TimeOuts.ReadIntervalTimeout = 100; //�������ʱ 1000
    TimeOuts.ReadTotalTimeoutMultiplier = 100; //��ʱ��ϵ�� 500
    TimeOuts.ReadTotalTimeoutConstant = 5000; //��ʱ�䳣��
    TimeOuts.WriteTotalTimeoutMultiplier = 500; // дʱ��ϵ��
    TimeOuts.WriteTotalTimeoutConstant = 2000; //дʱ�䳣��
    SetCommTimeouts(hCom, &TimeOuts);

    PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//��մ��ڻ�����

    memcpy(pHandle, &hCom, sizeof(hCom));//������

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
        // ͬ����ʽ
        DWORD dwBytesWrite = dat.length(); //�ɹ�д��������ֽ���
        BOOL bWriteStat = WriteFile(hCom, //���ھ��
            (char*)dat.c_str(), //�����׵�ַ
            dwBytesWrite, //Ҫ���͵������ֽ���
            &dwBytesWrite, //DWORD*���������շ��سɹ����͵������ֽ���
            NULL); //NULLΪͬ�����ͣ�OVERLAPPED*Ϊ�첽����
        if (!bWriteStat)
        {
            return 0;
        }
        return dwBytesWrite;
    }
    else
    {
        //�첽��ʽ
        DWORD dwBytesWrite = dat.length(); //�ɹ�д��������ֽ���
        DWORD dwErrorFlags; //�����־
        COMSTAT comStat; //ͨѶ״̬
        OVERLAPPED m_osWrite; //�첽��������ṹ��

                              //����һ������OVERLAPPED���¼��������������õ�����ϵͳҪ����ô��
        memset(&m_osWrite, 0, sizeof(m_osWrite));
        m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, "WriteEvent");

        ClearCommError(hCom, &dwErrorFlags, &comStat); //���ͨѶ���󣬻���豸��ǰ״̬
        BOOL bWriteStat = WriteFile(hCom, //���ھ��
            (char*)dat.c_str(), //�����׵�ַ
            dwBytesWrite, //Ҫ���͵������ֽ���
            &dwBytesWrite, //DWORD*���������շ��سɹ����͵������ֽ���
            &m_osWrite); //NULLΪͬ�����ͣ�OVERLAPPED*Ϊ�첽����
        if (!bWriteStat)
        {
            if (GetLastError() == ERROR_IO_PENDING) //�����������д��
            {
                WaitForSingleObject(m_osWrite.hEvent, 1000); //�ȴ�д���¼�1����
            }
            else
            {
                ClearCommError(hCom, &dwErrorFlags, &comStat); //���ͨѶ����
                CloseHandle(m_osWrite.hEvent); //�رղ��ͷ�hEvent�ڴ�
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
        //ͬ����ʽ
        DWORD wCount = 1024; //�ɹ���ȡ�������ֽ���
        BOOL bReadStat = ReadFile(hCom, //���ھ��
            buf, //�����׵�ַ
            wCount, //Ҫ��ȡ����������ֽ���
            &wCount, //DWORD*,�������շ��سɹ���ȡ�������ֽ���
            NULL); //NULLΪͬ�����ͣ�OVERLAPPED*Ϊ�첽����
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
        //�첽��ʽ
        DWORD wCount = 1024; //�ɹ���ȡ�������ֽ���
        DWORD dwErrorFlags;  //�����־
        COMSTAT comStat;     //ͨѶ״̬
        OVERLAPPED m_osRead; //�첽��������ṹ��

                             //����һ������OVERLAPPED���¼��������������õ�����ϵͳҪ����ô��
        memset(&m_osRead, 0, sizeof(m_osRead));
        m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, "ReadEvent");

        ClearCommError(hCom, &dwErrorFlags, &comStat); //���ͨѶ���󣬻���豸��ǰ״̬
        //if (!comStat.cbInQue)    
        //	return 0;  //������뻺�����ֽ���Ϊ0���򷵻�false

                       //std::cout << comStat.cbInQue << std::endl;
        BOOL bReadStat = ReadFile(hCom,     //���ھ��
            buf, //�����׵�ַ
            wCount, //Ҫ��ȡ����������ֽ���
            &wCount, //DWORD*,�������շ��سɹ���ȡ�������ֽ���
            &m_osRead); //NULLΪͬ�����ͣ�OVERLAPPED*Ϊ�첽����
        if (!bReadStat)
        {
            if (GetLastError() == ERROR_IO_PENDING) //����������ڶ�ȡ��
            {
                //GetOverlappedResult���������һ��������ΪTRUE
                //������һֱ�ȴ���ֱ����������ɻ����ڴ��������
                GetOverlappedResult(hCom, &m_osRead, &wCount, TRUE);
            }
            else
            {
                ClearCommError(hCom, &dwErrorFlags, &comStat); //���ͨѶ����
                CloseHandle(m_osRead.hEvent); //�رղ��ͷ�hEvent���ڴ�
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

//��ȫ������������ͨ��
WZSerialPort w;
//ÿ�ν���һ���ַ���Ϊ�뾶�������û�����
string buffer;
int radius_tmp = 5;
int get_turning_radius() {
    //Ԥ���ӿ�����׼��
    if(buffer.empty())buffer = w.receive();//��Ϊ��ʱ������ȡ���ˣ����ʱ����ȥȡ
    //���ʱ�򽲵���Ӧ����ȡ���ˣ���Ҫ��ûȡ�����Ǿͻ����
    if (buffer.size()) {
        //radius_tmp = buffer[0] - '0';
        radius_tmp = atoi(buffer.substr(0,3).c_str());
        buffer = "";
    }
    int radius = radius_tmp << 1;//tmp:cm -> mm -> pix *10 /5 <=> (<<1)
    return radius;
}

void ExpandCluster(unordered_map<int,int> &mp,int visited[],
    vector<int> &inCluster,int &index,vector<vector<p>> &cluster,
    vector<int> &tmp, vector<p>& points,float metric,float MinPts,int useful) {

    vector<p> newCluster;
    newCluster.push_back(points[index]);//��ڵĵ㣨���ĵ㣩�����µĴ���
    //printf("core point:%d\n", index);
    inCluster[index] = 1; //����ڴ���
    for (int i = 1; i < tmp.size();i++) { //������ܻ᲻�ϵ�����
        //i = 0 �ǿ϶��������
        visited[tmp[i]] = 1;
        vector<int> neighbor;
        neighbor.push_back(tmp[i]);
        /*for (int j = 0; j < tmp.size(); j++) { //������ܻ᲻�ϵ�����
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
                    tmp.push_back(n);//��չN����Ŀ
                    mp[n] = 1;//��ǽ���������
                }
            }
        }
        if (!inCluster[tmp[i]]) {//��������κδ���������´�
            newCluster.push_back(points[tmp[i]]);
            inCluster[tmp[i]] = 1; //��ǽ������
        }
    }
    cluster.push_back(newCluster);
}

/*
    int rbx = 320, rby = 240 + 260;
    �����������õĺ�������
    �뾶Ԥ����200
*/
int in_play;
inline bool indanger(float x, float y, float r,float turning_radius) {//�뾶�ӿ�
    if (abs(turning_radius) < 1e-7)return false;//ֱ��ȷ��û��Σ������
    int rbx = 320, rby = 240 + 300;//���ģ�320 240 ��װλ�õ�����1300mm/5 = 260pix ԭ��240+260
    int cxrb = rbx + turning_radius, cyrb = rby;//�Һ��ֵ�ת��Բ������
    
    float dist2 = sqrt((x - cxrb) * (x - cxrb) + (y - cyrb) * (y - cyrb));
    if (dist2 >= turning_radius - r && y <= cyrb) { 
        return true; 
    }//��֤���ڵڶ������� �뾶�ӿ�
    return false;
}

int get_lspeed() {//���ٽӿ�
    int speed = 50;
    return speed;//����
}

int get_rspeed() {//���ٽӿ�
    int speed = 50;
    return speed;//����
}

void DBSCAN(int visited[],vector<p>& points,int useful,float Eps,float MinPts,int turning_radius) {
    int num = 0;//�صĸ���
    int inner = 0; //���ڲ���ĸ��� ���ڱ���ǲ�������
    vector<int> isNoise(useful);//���ڱ���Ƿ�������
    vector<vector<p>> Cluster;
    vector<int> inCluster(useful);
    unordered_map<int, int> mp; //���Ա���Ƿ���������

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
    //�������������뾶
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
         //����/5 ʹ����ʵ����mm�ܹ�ת����pix
         centerX = centerX /5 + 320, centerY = centerY /5 + 240;//��ƫ����
         //printf("%d:horizon:%f vertical:%f radius:%f\n", i,centerX,centerY, radius);//����������֮��Ӧ��û����
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
        //w.send("1");//1��ΪΣ�ձ�־
    }
    else {
        //w.send("0");//0��Ϊ��ȫ��־
        in_play = 0;
        PlaySound(NULL, NULL, SND_FILENAME | SND_ASYNC | SND_LOOP);
    }
}

void stm32_init() {
    DCB dcb;
    HANDLE m_hCom;//COM�ھ�� 
    m_hCom = CreateFile(
        "COM9",//Ԥ��COM9
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL);
    if (m_hCom != INVALID_HANDLE_VALUE) // �򿪴��пڳɹ� 
    {
        // ����ͨѶ���� 
        GetCommState(m_hCom, &dcb);


        dcb.BaudRate = CBR_9600;//������ 
        dcb.Parity = NOPARITY;
        //EVENPARITY żУ�� ,NOPARITY ��У�� 
        //MARKPARITY ���У��   ODDPARITY ��У�� 
        dcb.ByteSize = 8;//����λ 
        dcb.StopBits = ONESTOPBIT;// ONESTOPBIT 1λֹͣλ   
                                  //TWOSTOPBITS 2λֹͣλ 
                                  //ONE5STOPBITS   1.5λֹͣλ 

        COMMTIMEOUTS m_CommTimeouts;
        SetCommState(m_hCom, &dcb);
        // Config timeouts���ó�ʱ 
        m_CommTimeouts.ReadIntervalTimeout = 0;
        m_CommTimeouts.ReadTotalTimeoutConstant = 1000;
        m_CommTimeouts.ReadTotalTimeoutMultiplier = 0;
        m_CommTimeouts.WriteTotalTimeoutConstant = 200;
        m_CommTimeouts.WriteTotalTimeoutMultiplier = 0;
        SetCommTimeouts(m_hCom, &m_CommTimeouts);
        // Clear buffer��������� 
        PurgeComm(m_hCom, PURGE_TXABORT);
        PurgeComm(m_hCom, PURGE_RXABORT);
        PurgeComm(m_hCom, PURGE_TXCLEAR);
        PurgeComm(m_hCom, PURGE_RXCLEAR);

        // �򿪴��пڳɹ� 
        //return TRUE;
    }
    
}

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
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
        opt_com_path = "\\\\.\\com3";
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

        /*
            Area:Canvas setup
        */
        //�������
        //initgraph(640, 480); //put on a canvas
        bool flag = w.open("com4");
        /*if (!flag) {
            printf("cannot open com4");
            return 0;
        }*/
        initgraph(1280, 960);//�޸Ļ�����С֮���Ƿ�Ӧ����������centerX��centerY��
        int radius = 200; //radius of circle
        int centerX = 320, centerY = 240; //center of circle
        //circle(centerX,centerY,radius);
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
        loadimage(&img,"../img/car_3.jpg",160,500);//ʵ���޸� ʵ���ߴ� ��1.80m ��������ࣩ0.75m
        //ʵ���ߴ� ��:2.50m ��0.80m
        //2500/5 = 500 800/5 = 160
        in_play = 0;//������־��ʼ��
        putimage(160, 80, &img);//��ʾλ�ñ궨 320 - 160 
        //��������Ҫ����800/5 = 160 240-160 = 80
        setcolor(RED);//�ϰ�˵�ĳɺ�ɫ
        settextcolor(BLACK);//����������ɫ
        LOGFONT f;
        gettextstyle(&f);						// ��ȡ��ǰ��������
        f.lfHeight = 30;						// ��������߶�Ϊ 48
        _tcscpy_s(f.lfFaceName, _T("����"));		// ��������Ϊ�����塱(�߰汾 VC �Ƽ�ʹ�� _tcscpy_s ����)
        f.lfQuality = ANTIALIASED_QUALITY;		// �������Ч��Ϊ�����  
        settextstyle(&f);						// ����������ʽ
        
        /*outtextxy(0, 35, _T("������:"));
        outtextxy(0, 70, _T("������:"));*/
        int rbx = 320, rby = 240 + 300;//��װλ��+��װλ�õ����ֳ��� ԭ240+260 ���ڶ�ƫ��һ��
        int turning_radius = get_turning_radius();
        int turning_left_speed = get_lspeed();
        int turning_right_speed = get_rspeed();
        char r[10], ls[10], rs[10];
        if (turning_radius) {
            outtextxy(0, 0, _T("�뾶Ϊ:"));
            sprintf_s(r, "%d", turning_radius * 5);//ת����mm
            outtextxy(100, 0, r);
        }
        else {
            outtextxy(0, 0, _T("����ת"));
        }
        /*sprintf_s(ls, "%d", turning_left_speed);
        sprintf_s(rs, "%d", turning_right_speed);*/
        /*outtextxy(100,35, ls);
        outtextxy(100, 70, rs);*/
        if(turning_radius) arc(rbx, rby-turning_radius, rbx+(turning_radius<<1), rby+turning_radius, PI / 2, -PI);//�뾶�ӿ�
        //��ת��뾶��0�򲻻�
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
                    //��ת��Ϊ�����ϵ�����֮������򲻹�
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

                DBSCAN(visited, tmp, useful, 150, 10,turning_radius); //��ת��뾶Ҳ���� ԭ��150 15
                Sleep(200);//�ӳ�ʱ�䣿

                turning_radius = get_turning_radius();//ͨ���ӿ�ˢ�°뾶
                turning_left_speed = get_lspeed();
                turning_right_speed = get_rspeed();
                
                cleardevice();
                putimage(160, 80, &img);
                setcolor(BLACK);
                if (turning_radius) {
                    outtextxy(0, 0, _T("�뾶Ϊ:"));
                    sprintf_s(r, "%d", turning_radius * 5);//ת����mm
                    outtextxy(100, 0, r);
                }
                else {
                    outtextxy(0, 0, _T("����ת"));
                }
                /*outtextxy(0, 35, _T("������:"));
                outtextxy(0, 70, _T("������:"));*/
                /*sprintf_s(ls, "%d", turning_left_speed);
                sprintf_s(rs, "%d", turning_right_speed);*/
                /*outtextxy(100, 35, ls);
                outtextxy(100, 70, rs);*/
                setcolor(RED);
                if (turning_radius != 0) arc(rbx, rby - turning_radius, rbx + (turning_radius << 1), rby + turning_radius, PI / 2, -PI);//�뾶�ӿ�

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

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}

