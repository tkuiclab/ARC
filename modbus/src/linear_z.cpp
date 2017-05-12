#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
// #include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "modbus/LM_Cmd.h"
#include "modbus/modbus.h"

#define LOOP 1
#define USB0 0
#define USB1 1
#define ADDRESS_START 0
#define ADDRESS_END 99
// #define position

std_msgs::Int32 std_msg;
std_msgs::Int32 feedback;
ros::Publisher feedback_pub;
modbus::LM_Cmd LM_Msg;
bool pub = false;
modbus_t *ctx, *ctz, *tmp_ct;



void first_topic_callback(const modbus::LM_Cmd::ConstPtr &tmp_LM_Msg)
{
    ROS_INFO("position: [%i]", tmp_LM_Msg->x);
    LM_Msg.x = tmp_LM_Msg->x;
    LM_Msg.z = tmp_LM_Msg->z;
    LM_Msg.id = tmp_LM_Msg->id;
    pub = true;
}
modbus_t* Init_Modus_RTU(bool &Is_Success, int ID, std::string Port, int BaudRate)
{
    modbus_t*ct = modbus_new_rtu(Port.c_str(), 9600, 'E', 8, ID);
    modbus_set_slave(ct, ID);
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
        int up_pos = pos-65535;
        if(up_pos<=0)
        {
            rc = modbus_write_register(ctx, 6146, 0);
            rc = modbus_write_register(ctx, 6147, pos);
        }
        else
        {
            rc = modbus_write_register(ctx, 6146, 1);
            rc = modbus_write_register(ctx, 6147, up_pos);
            std::cout<<"up pos = "<<up_pos<<"\n";
        }

        //速度
        rc = modbus_write_register(ctx, 6148, 0);
        rc = modbus_write_register(ctx, 6149, 3000);

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

void callback1(const ros::TimerEvent&)
{
  if((LM_Msg.id == 1)||(LM_Msg.id == 3))   
    {
        SendCmd(true, ctx, LM_Msg.x);  //ctx_ID_1  USB0  right
        ROS_INFO("Callback 1 triggered");
    }
}

void callback2(const ros::TimerEvent&)
{
    if((LM_Msg.id == 2)||(LM_Msg.id == 3))   
    {
        SendCmd(true, ctz, LM_Msg.z);  //ctz_ID_2  USB1  left
        ROS_INFO("Callback 2 triggered");
    }
}

int main(int argc, char **argv)
{
    // For Allocate_and_Init_MemorySpace
    uint16_t *tab_rq_registers;
    uint16_t *tab_rp_registers;

    //========================= Initialize ROS =============================
    ros::init(argc, argv, "linear_z");
    
    ros::NodeHandle nh_param("~");
    std::string port_x;
    std::string port_z;
    int  baud_rate;

    nh_param.param<std::string>("port_x", port_x,"ttyUSB0");
    nh_param.param<std::string>("port_z", port_z,"ttyUSB1");
    
    nh_param.param<int>("baud", baud_rate, 9600);
    // std::cout<<port<<"\n";

    //========================= Initialize Modbus_RTU ============================= 
    bool Connect_X_OK = false;
    bool Connect_Z_OK = false;
    ctx = Init_Modus_RTU(Connect_X_OK, 1, "/dev/ttyUSB0", 9600);
    ctz = Init_Modus_RTU(Connect_Z_OK, 2, "/dev/ttyUSB1", 9600);
    
    if((Connect_X_OK == false)||(Connect_X_OK == false))
    {
        if(Connect_X_OK == false)   std::cout<<"CONNECT X ERROR!!!!!\n";
        if(Connect_Z_OK == false)   std::cout<<"CONNECT Z ERROR!!!!!\n";
        return -1;
    }
    std::cout<<"Connect_X_OK = "<<Connect_X_OK<<"\n";
    std::cout<<"Connect_Z_OK = "<<Connect_Z_OK<<"\n";

    // ============================= Subscribe message =============================
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/position_topic", 10, first_topic_callback);
    ros::Rate loop_rate(10);

    // ============================= ROS Loop =============================

    while (ros::ok())
    {
        if((LM_Msg.id == 1)||(LM_Msg.id == 3))   
        {
            tmp_ct = ctx;
            SendCmd(true, ctx, LM_Msg.x);
        }
        if(LM_Msg.id == 2)
        {
            tmp_ct = ctz;
            SendCmd(true, ctz, LM_Msg.z);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
