#ifndef _MULTI_H_
#define _MULTI_H_

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstring>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <chassis_msgs/chassis.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tcp_msgs/tcp_ctrl.h>


using namespace std;


#define TCP_ADDR            "10.0.0.75"
//#define TCP_ADDR            "127.0.0.1"
#define TCP_PORT            1234


#define BUFFER_MAX_SIZE     512     /* 最大的缓冲区大小 */
#define POINT_MAX_SIZE      32      /* 最多导航点个数   */



class DataProcess{
public:
	ros::NodeHandle nh;
    ros::Subscriber chassis_sub;
    ros::Subscriber pose_sub;

    ros::Publisher  tcp_pub;
    

public:
	DataProcess();

    // 回调函数1,作为类的成员函数
    void chassis_cb(const chassis_msgs::chassis& msg);
    void pose_cb(const geometry_msgs::PoseWithCovarianceStamped& msg);
};

typedef union{
    unsigned char   byte[4];
    float           value;
}float_union_t;

typedef struct{

    float_union_t   x;      /* 航点 x */
    float_union_t   y;      /* 航点 y */
    float           t;      /* 航点 姿态 */
} point_t;

typedef struct{
    float_union_t v;
    float_union_t w;
} velocity_t;

typedef struct{
    float_union_t x;
    float_union_t y;
} position_t;

typedef enum {
    RUNNING,
    PAUSE,
    STOP,
} state_t;


typedef struct {
    state_t     action;
    int         point_num;
    point_t     *point;
    int         point_flag;

    velocity_t velocity;
}
recv_data_t;

typedef struct {
    uint8_t mode;
    velocity_t velocity;
    position_t position;
    float_union_t   orientation;
} chassis_t;



#define FRAME_HEAD1                 0
#define FRAME_HEAD2                 1
#define FRAME_ADDR1                 2
#define FRAME_ADDR2                 3
#define FRAME_NUMB1                 4
#define FRAME_NUMB2                 5
#define FRAME_NUMB3                 6
#define FRAME_NUMB4                 7
#define FRAME_TYPE1                 8
#define FRAME_COMD1                 9
/* N 为总的数据的个数，仅数据段，n为第n个数据，0为第一个， */
#define FRAME_DATA(n)               (n + 10)
#define FRAME_CRC1(N)               (N + 10)
#define FRAME_CRC2(N)               (N + 11)
#define FRAME_TAIL1(N)              (N + 12)
#define FRAME_TAIL2(N)              (N + 13)

#define SEND_LENGTH(N)              (N + 14)

#define CRC_START                   (&send_frame[2])
#define CRC_LENGTH(N)               (8 + N)






#endif