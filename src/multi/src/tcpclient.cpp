#include <ros/ros.h>
#include <iostream>
#include <cstring>

#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
// #include <tcpclient/tcpclient.h>

#include <pthread.h>

#include "multi.h"


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define     MOVEBASE_GOAL_MAX       32

move_base_msgs::MoveBaseGoal goal[MOVEBASE_GOAL_MAX];

uint8_t point_flag = 0;


/*-----------------------------------  数据相关定义  -----------------------------------*/

/*-----------------------------------  协议相关定义  -----------------------------------*/

/* 网络通信相关 */

int                 sock;
struct sockaddr_in  serv_addr;

unsigned short crc16_xmodem(unsigned char *ptr, int len);

/*----------------------------- 接收上位机数据处理线程 ---------------------------------------*/

int point_nums = 0;                                                     /* 接收的导航点的个数 */
point_t point[POINT_MAX_SIZE];                                          /* 存放收到的导航点 */

unsigned char *byte;                                                    /* 解析导航点用 */

velocity_t velocity_cmd;

recv_data_t recv_data = {
    .action = STOP,
};

void *tcp_recv_thread_entry(void *arg)
{
    ros::Rate loop_rate(20);

    /* 接收控制帧 */
    unsigned char       recv_frame[BUFFER_MAX_SIZE];        /* 通信数据缓冲区 */
    unsigned short      crc_check;

    /* 发送反馈帧 */
    unsigned char       send_frame[BUFFER_MAX_SIZE];        /* 通信数据缓冲区 */

    /* 反馈帧：包头 */
    send_frame[FRAME_HEAD1] = 0xFC;
    send_frame[FRAME_HEAD2] = 0x03;

    /* 反馈帧：地址，目前不用，全置零 */
    send_frame[FRAME_ADDR1] = 0x00;
    send_frame[FRAME_ADDR2] = 0x00;

    point[0].x.value = 0;
    point[0].y.value = 0;
    point[0].t       = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        memset(recv_frame, 0x00, sizeof(recv_frame));
        read(sock, recv_frame, sizeof(recv_frame)-1);                               /* 从服务器读取数据 */

        if (recv_frame[0] == 0xFC && recv_frame[1] == 0x03)
        {
            switch (recv_frame[FRAME_COMD1])      /* 指令处理 */
            {
                case 0xA1:  /* 上位机直接速度控制 */

                    /* 数据解包处理 */
                    byte = &recv_frame[10];

                    velocity_cmd.v.byte[0] = byte[3];
                    velocity_cmd.v.byte[1] = byte[2];
                    velocity_cmd.v.byte[2] = byte[1];
                    velocity_cmd.v.byte[3] = byte[0];

                    velocity_cmd.w.byte[0] = byte[7];
                    velocity_cmd.w.byte[1] = byte[6];
                    velocity_cmd.w.byte[2] = byte[5];
                    velocity_cmd.w.byte[3] = byte[4];
                    
                    recv_data.velocity.v.value = velocity_cmd.v.value;
                    recv_data.velocity.w.value = velocity_cmd.w.value;

                    ROS_INFO("v %f  w %f", recv_data.velocity.v.value, recv_data.velocity.w.value);

                    /* 反馈帧：消息长度，高位在前 */
                    send_frame[FRAME_NUMB1] = 0x00;
                    send_frame[FRAME_NUMB2] = 0x00;
                    send_frame[FRAME_NUMB3] = 0x00;
                    send_frame[FRAME_NUMB4] = 0 + 4;

                    /* 反馈帧：类型 */
                    send_frame[FRAME_TYPE1] = 0x12;

                    /* 反馈帧：指令 */
                    send_frame[FRAME_COMD1] = 0xA1;
                                        
                    crc_check = crc16_xmodem(CRC_START, CRC_LENGTH(0));

                    send_frame[FRAME_CRC1(0)] = ((unsigned char *)&crc_check)[1];
                    send_frame[FRAME_CRC2(0)] = ((unsigned char *)&crc_check)[0];

                    /* 反馈帧：包尾 */
                    send_frame[FRAME_TAIL1(0)] = 0xEB;
                    send_frame[FRAME_TAIL2(0)] = 0x90;

                    write(sock, send_frame, SEND_LENGTH(0));
                break;

            case 0xA7:  /* 上位机发送航点 */

                /* 数据解包处理 */
                recv_data.action = (state_t)recv_frame[10];     /* 上位机的 action 指令 */

                if (recv_frame[10] == 1)                        /* 仅在 1 时(开始)处理航点数据 */
                {
                    point_nums = recv_frame[11];
                    byte = &recv_frame[12];

                    for (int i=1; i<= point_nums; i++)
                    {
                        point[i].x.byte[0] = byte[(i-1)*8+3];
                        point[i].x.byte[1] = byte[(i-1)*8+2];
                        point[i].x.byte[2] = byte[(i-1)*8+1];
                        point[i].x.byte[3] = byte[(i-1)*8+0];

                        point[i].y.byte[0] = byte[(i-1)*8+7];
                        point[i].y.byte[1] = byte[(i-1)*8+6];
                        point[i].y.byte[2] = byte[(i-1)*8+5];
                        point[i].y.byte[3] = byte[(i-1)*8+4];

                        point[i].t = atan2(point[i].y.value - point[i-1].y.value, point[i].x.value - point[i-1].x.value);
                    }

                    point_flag = 1;

                    ROS_INFO("point num: %d", point_nums);
                    for (int i=0; i<= point_nums; i++)
                    {
                        ROS_INFO("point %d : (%f ,%f, %f)", i, point[i].x.value, point[i].y.value, point[i].t);
                    }
                }

                /* 反馈帧：消息长度，高位在前 */
                send_frame[FRAME_NUMB1] = 0x00;
                send_frame[FRAME_NUMB2] = 0x00;
                send_frame[FRAME_NUMB3] = 0x00;
                send_frame[FRAME_NUMB4] = 1 + 4;

                /* 反馈帧：类型 */
                send_frame[FRAME_TYPE1] = 0x12;

                /* 反馈帧：指令 */
                send_frame[FRAME_COMD1] = 0xA7;

                send_frame[FRAME_DATA(0)] = recv_frame[10];

                crc_check = crc16_xmodem(CRC_START, CRC_LENGTH(1));

                send_frame[FRAME_CRC1(1)] = ((unsigned char *)&crc_check)[1];
                send_frame[FRAME_CRC2(1)] = ((unsigned char *)&crc_check)[0];

                /* 反馈帧：包尾 */
                send_frame[FRAME_TAIL1(1)] = 0xEB;
                send_frame[FRAME_TAIL2(1)] = 0x90;
                
                write(sock, send_frame, SEND_LENGTH(1));                

            break;

            default:
                ROS_INFO("unkonw command");
                break;
            }
        }
        else
        {
            ROS_INFO("frame header error!");
        }
        
        loop_rate.sleep();
    }
}

/*---------------------------------------- 任务处理线程 ----------------------------------*/


state_t state = STOP;   

void *mission_thread_entry(void *arg)
{
    ros::Rate loop_rate(4);

    while(ros::ok())
    {
        switch (state)
        {
            case RUNNING:

                /* 状态行为 */
                // 调用运行时的函数
                // 当前导航点，



                /* 状态转移 */
                if (recv_data.action == 2)  /* action 2 暂停 */
                {
                    state = PAUSE;
                }
                if (recv_data.action == 0)  /* action 0 停止 */
                {
                    state = STOP;
                }

            break;

            case STOP:
                //调用停止函数

                /* 状态转移 */
                if (recv_data.action == 1)  /* action 1 开始 */
                {
                    state = RUNNING;
                } 
            break;

            case PAUSE:

                //调用暂停 函数


                /* 状态转移 */
                if (recv_data.action == 3) /* action 3 继续 */
                {
                    state = RUNNING;
                }
                if (recv_data.action == 0) /* action 0 停止 */
                {
                    state = STOP;
                }            

            break;

            default:
                break;
        }

        // ROS_INFO("state : %d ", state);

        loop_rate.sleep();
    }
}


void *running_thread_entry(void *arg)
{
    int goal_number = 0;

    ros::Rate loop_rate(10);

    MoveBaseClient ac("move_base", true);

    // while(!ac.waitForServer( ros::Duration( 2.0 ) )){
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }

    while(ros::ok())
    {
        if (point_flag == 1) /* 有新的导航点到了 */
        {
            goal_number = point_nums;

            /* 收到的点转换为movebase的导航点 */
            for (int i=0; i<point_nums; i++)
            {
                goal[i].target_pose.pose.position.x     = point[i+1].x.value;
                goal[i].target_pose.pose.position.y     = point[i+1].y.value;
                goal[i].target_pose.pose.orientation.w  = cos(point[i+1].t/2.0);
                goal[i].target_pose.pose.orientation.z  = sin(point[i+1].t/2.0);

                ROS_INFO("goal %d : (%f ,%f, %f) (%f + %f z)", i, 
                        (float)goal[i].target_pose.pose.position.x, 
                        (float)goal[i].target_pose.pose.position.y,
                        (float)point[i+1].t,
                        (float)goal[i].target_pose.pose.orientation.w, 
                        (float)goal[i].target_pose.pose.orientation.z);
            }

            /* 开始导航 */
            for (int i=0; i<point_nums; i++)
            {
                goal[i].target_pose.header.frame_id = "map";
                goal[i].target_pose.header.stamp = ros::Time::now();
                ac.sendGoal(goal[i]);
            
                ROS_INFO("Send NO. %d Goal !!!", i+1);

                ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("The NO. %d Goal achieved success !!!", i+1 );
                }
                else
                {
                    ROS_WARN("The NO. %d Goal Planning Failed for some reason", i+1); 
                }
            }
            point_flag = 0;        /* 处理结束，等待新任务 */
        }
        loop_rate.sleep();
    }
}

/*--------------------------------- 底盘状态上报线程 -----------------------------------*/

DataProcess::DataProcess(){
    chassis_sub = nh.subscribe("chassis", 5, &DataProcess::chassis_cb, this);
    pose_sub = nh.subscribe("/car1/amcl_pose", 5, &DataProcess::pose_cb, this);

    tcp_pub = nh.advertise<tcp_msgs::tcp_ctrl>("tcp_ctrl_pub", 30);

}

chassis_t chassis_info;

/* 底盘发回来的速度 */
void DataProcess::chassis_cb(const chassis_msgs::chassis& msg)
{
    chassis_info.mode = msg.mode;

    chassis_info.velocity.v.value = msg.velocity_l;
    chassis_info.velocity.w.value = msg.velocity_w;

    // ROS_INFO("mode %d", msg.mode);
    // ROS_INFO("v %f", msg.velocity_l);
    // ROS_INFO("w %f", msg.velocity_w);
}

/* 位姿信息 */
void DataProcess::pose_cb(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    chassis_info.position.x.value = msg.pose.pose.position.x;
    chassis_info.position.y.value = msg.pose.pose.position.y;
    chassis_info.orientation.value = msg.pose.pose.orientation.w;

    // ROS_INFO("x: %f  y: %f", chassis_info.location.x.value, chassis_info.location.y.value);
}


void *tcp_send_thread(void *arg)
{
    ros::Rate loop_rate(1);    

    unsigned char       send_frame[BUFFER_MAX_SIZE];        /* 通信数据缓冲区 */
    unsigned short      crc_check;

    /* 数据上报帧，通信协议：包头 */
    send_frame[FRAME_HEAD1] = 0xFC;
    send_frame[FRAME_HEAD2] = 0x03;

    /* 通信协议：地址，目前不用，全置零 */
    send_frame[FRAME_ADDR1] = 0x00;
    send_frame[FRAME_ADDR2] = 0x00;  

    while(ros::ok())
    {
        /* ------------------------------------------------------ 状态上报 */

        /* 通信协议：消息长度，高位在前 */
        send_frame[FRAME_NUMB1] = 0x00;
        send_frame[FRAME_NUMB2] = 0x00;
        send_frame[FRAME_NUMB3] = 0x00;
        send_frame[FRAME_NUMB4] = 1 + 4;

        /* 通信协议：类型 */
        send_frame[FRAME_TYPE1] = 0x12;

        /* 通信协议：指令，状态上报 0xA0 */
        send_frame[FRAME_COMD1] = 0xA0;

        send_frame[FRAME_DATA(0)] = chassis_info.mode;
             
        crc_check = crc16_xmodem(CRC_START, CRC_LENGTH(1));

        send_frame[FRAME_CRC1(1)] = ((unsigned char *)&crc_check)[1];
        send_frame[FRAME_CRC2(1)] = ((unsigned char *)&crc_check)[0];

        /* 通信协议：包尾 */
        send_frame[FRAME_TAIL1(1)] = 0xEB;
        send_frame[FRAME_TAIL2(1)] = 0x90;

        write(sock, send_frame, SEND_LENGTH(1));

        // ROS_INFO("mode %d", chassis_info.mode);        
        
        loop_rate.sleep();

        /* ------------------------------------------------------ 位置上报 */

        /* 通信协议：消息长度，高位在前 */
        send_frame[FRAME_NUMB1] = 0x00;
        send_frame[FRAME_NUMB2] = 0x00;
        send_frame[FRAME_NUMB3] = 0x00;
        send_frame[FRAME_NUMB4] = 16 + 4;

        /* 通信协议：类型 */
        send_frame[FRAME_TYPE1] = 0x12;

        /* 通信协议：指令，状态上报 0xA0 */
        send_frame[FRAME_COMD1] = 0xA4;

        send_frame[FRAME_DATA(0)] = chassis_info.position.x.byte[3];
        send_frame[FRAME_DATA(1)] = chassis_info.position.x.byte[2];
        send_frame[FRAME_DATA(2)] = chassis_info.position.x.byte[1];
        send_frame[FRAME_DATA(3)] = chassis_info.position.x.byte[0];

        send_frame[FRAME_DATA(4)] = chassis_info.position.y.byte[3];
        send_frame[FRAME_DATA(5)] = chassis_info.position.y.byte[2];
        send_frame[FRAME_DATA(6)] = chassis_info.position.y.byte[1];
        send_frame[FRAME_DATA(7)] = chassis_info.position.y.byte[0];

        send_frame[FRAME_DATA(8)] = chassis_info.velocity.v.byte[3];
        send_frame[FRAME_DATA(9)] = chassis_info.velocity.v.byte[2];
        send_frame[FRAME_DATA(10)] = chassis_info.velocity.v.byte[1];
        send_frame[FRAME_DATA(11)] = chassis_info.velocity.v.byte[0];

        send_frame[FRAME_DATA(12)] = chassis_info.velocity.w.byte[3];
        send_frame[FRAME_DATA(13)] = chassis_info.velocity.w.byte[2];
        send_frame[FRAME_DATA(14)] = chassis_info.velocity.w.byte[1];
        send_frame[FRAME_DATA(15)] = chassis_info.velocity.w.byte[0];

        crc_check = crc16_xmodem(CRC_START, CRC_LENGTH(16));

        send_frame[FRAME_CRC1(16)] = ((unsigned char *)&crc_check)[1];
        send_frame[FRAME_CRC2(16)] = ((unsigned char *)&crc_check)[0];

        /* 通信协议：包尾 */
        send_frame[FRAME_TAIL1(16)] = 0xEB;
        send_frame[FRAME_TAIL2(16)] = 0x90;

        write(sock, send_frame, SEND_LENGTH(16));

        loop_rate.sleep();

        /* ------------------------------------------------------ 进度上报 */
        
        /* 通信协议：消息长度，高位在前 */
        send_frame[FRAME_NUMB1] = 0x00;
        send_frame[FRAME_NUMB2] = 0x00;
        send_frame[FRAME_NUMB3] = 0x00;
        send_frame[FRAME_NUMB4] = 1 + 4;

        /* 通信协议：类型 */
        send_frame[FRAME_TYPE1] = 0x12;

        /* 通信协议：指令，状态上报 0xA6 */
        send_frame[FRAME_COMD1] = 0xA6;

        send_frame[FRAME_DATA(0)] = 0;  /* 进度 */

        crc_check = crc16_xmodem(CRC_START, CRC_LENGTH(1));

        send_frame[FRAME_CRC1(1)] = ((unsigned char *)&crc_check)[1];
        send_frame[FRAME_CRC2(1)] = ((unsigned char *)&crc_check)[0];

        /* 通信协议：包尾 */
        send_frame[FRAME_TAIL1(1)] = 0xEB;
        send_frame[FRAME_TAIL2(1)] = 0x90;

        write(sock, send_frame, SEND_LENGTH(1)); 

        loop_rate.sleep();
    }
}

/*--------------------------------- 主线程 -----------------------------------*/

int main(int argc, char** argv){
    
    ros::init(argc, argv, "multi");

    DataProcess p;

    /*----------------------------------------- TCP 客户端初始化的操作 */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;  
    serv_addr.sin_addr.s_addr = inet_addr(TCP_ADDR);                        //服务器 IP 地址
    serv_addr.sin_port = htons(TCP_PORT);                                   //端口
    connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    
    /*-------------------------------------- 创建接收线程、发送线程、任务处理线程 */
    int res;
    pthread_t tcprecv_thread, mission_thread, tcpsend_thread;
    void* thread_result;

    res = pthread_create(&tcprecv_thread, NULL, tcp_recv_thread_entry, NULL);
    res = pthread_create(&mission_thread, NULL, mission_thread_entry, NULL);
    res = pthread_create(&tcpsend_thread, NULL, tcp_send_thread, NULL);

    /* RUNNING 控制线程 */

    pthread_t running_thread;

    res = pthread_create(&running_thread, NULL, running_thread_entry, NULL);

    /* 等待线程结束 */
    pthread_join(tcprecv_thread, &thread_result);
    pthread_join(mission_thread, &thread_result);
    pthread_join(tcpsend_thread, &thread_result);

    //关闭套接字
    close(sock);

    return 0;  
}


unsigned short crc16_xmodem(unsigned char *ptr, int len)
{
    unsigned int i;
    unsigned short crc = 0x0000;
    
    while(len--)
    {
        crc ^= (unsigned short)(*ptr++) << 8;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}


