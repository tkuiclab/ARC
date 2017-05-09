#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include "modbus/modbus.h"

#define LOOP 1
#define ID_LINEAR_X 1
#define ID_LINEAR_Z 2
#define ADDRESS_START 0
#define ADDRESS_END 99
// #define position

std_msgs::Int32 std_msg;
std_msgs::Int32 feedback;
ros::Publisher feedback_pub;
bool pub = false;

void first_topic_callback(const std_msgs::Int32::ConstPtr &msg)
{
    ROS_INFO("position: [%i]", msg->data);
    std_msg.data = msg->data;
    pub = true;
}

modbus_t* Init_Modus_RTU(bool &Is_Success, int ID, std::string Port, int BaudRate)
{
    modbus_t* ct = modbus_new_rtu("/dev/ttyUSB0", 9600, 'E', 8, 2);
    modbus_set_slave(ct, 2);
    if (modbus_connect(ct) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n",
        modbus_strerror(errno));
        std::cout<<"ERR connect\n";
        modbus_free(ct);
        Is_Success = false;
    }
    else
    {
        std::cout<<"init ok\n";
        Is_Success = true;
    }
    return ct;
}

void SendCmd(bool Is_Pub, modbus_t* ctx, int pos)
{
    int rc;
    if (pub == true)
    {
        //輸入寫入
        rc = modbus_write_register(ctx, 125, 0);

        //運轉方式
        rc = modbus_write_register(ctx, 6144, 0);
        rc = modbus_write_register(ctx, 6145, 1);

        //位置
        rc = modbus_write_register(ctx, 6146, 0);
        rc = modbus_write_register(ctx, 6147, std_msg.data);

        //速度
        rc = modbus_write_register(ctx, 6148, 0);
        rc = modbus_write_register(ctx, 6149, 1500);

        //起動
        rc = modbus_write_register(ctx, 6150, 0);
        rc = modbus_write_register(ctx, 6151, 700000);

        //停止
        //rc = modbus_write_register(ctx, 6152, 0);
        //printf("6152 rc=%d\n",rc);
        //rc = modbus_write_register(ctx, 6153, 700000);
        //printf("6153 rc=%d\n",rc);

        //運轉電流
        rc = modbus_write_register(ctx, 6154, 0);
        rc = modbus_write_register(ctx, 6155, 500);

        //結合
        rc = modbus_write_register(ctx, 6158, 0);
        rc = modbus_write_register(ctx, 6159, 0);

        //輸入啟動
        rc = modbus_write_register(ctx, 125, 8);

        //輸出結束
        rc = modbus_write_register(ctx, 127, 8);

        pub = false;
    }
}

int main(int argc, char **argv)
{
    modbus_t *ctx, *ctz;

    // For Allocate_and_Init_MemorySpace
    uint16_t *tab_rq_registers;
    uint16_t *tab_rp_registers;

    //========================= Initialize ROS =============================
    ros::init(argc, argv, "linear_z");
    
    ros::NodeHandle nh_param("~");
    std::string port;
    int  baud_rate;

    nh_param.param<std::string>("port", port,"ttyUSB0");
    nh_param.param<int>("baud", baud_rate, 9600);
    
    //========================= Initialize Modbus_RTU ============================= 
    bool IsConnect_ctx = true;
    ctx = Init_Modus_RTU(IsConnect_ctx, 2, "/dev/ttyUSB0", 9600);
    if(IsConnect_ctx == false) 
    {
        std::cout<<"CONNECT ERROR!!!!!\n";
        ROS_INFO("Connect ERROR!!!");
        return -1;
    }
    else
        std::cout<<"Connect port : "<<port<<"\n";

    // ============================= Subscribe message =============================
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/position_topic", 10, first_topic_callback);
    ros::Rate loop_rate(10);

    // ============================= ROS Loop =============================
    while (ros::ok())
    {
        //GetFeedBack(ctx, tab_rp_registers, tab_rq_registers);
        SendCmd(pub, ctx, std_msg.data);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
