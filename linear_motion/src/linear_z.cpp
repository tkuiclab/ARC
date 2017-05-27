#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
// #include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "linear_motion/LM_Cmd.h"
#include "modbus/modbus.h"

#define LOOP 1
#define MODE_GET_CURR_POS 291
#define ADDRESS_START 0
#define ADDRESS_END 99
// #define position

std_msgs::Int32 std_msg;
std_msgs::Int32 feedback;
ros::Publisher feedback_pub;
linear_motion::LM_Cmd LM_Msg;
bool pub = false;
modbus_t *ctx, *ctz, *tmp_ct;
int curr_state = 10;

void first_topic_callback(const linear_motion::LM_Cmd::ConstPtr &tmp_LM_Msg)
{
    ROS_INFO("position: [%i]", tmp_LM_Msg->x);
    LM_Msg.x = tmp_LM_Msg->x;
    LM_Msg.z = tmp_LM_Msg->z;
    LM_Msg.id = tmp_LM_Msg->id;
    // LM_Msg.is_busy = tmp_LM_Msg->isbusy;
    // LM_Msg.curr_pos = tmp_LM_Msg->curr_pos;
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

void SendCmd(bool Is_Pub, modbus_t* ct, int pos)
{
    int rc;
    if (pub == true)
    {
        //輸入寫入
        rc = modbus_write_register(ct, 125, 0);

        //運轉方式
        rc = modbus_write_register(ct, 6144, 0);
        rc = modbus_write_register(ct, 6145, 1);

        //位置
        int up_pos = pos-65535;
        if(up_pos<=0)
        {
            rc = modbus_write_register(ct, 6146, 0);
            rc = modbus_write_register(ct, 6147, pos);
        }
        else
        {
            rc = modbus_write_register(ct, 6146, 1);
            rc = modbus_write_register(ct, 6147, up_pos);
            std::cout<<"up pos = "<<up_pos<<"\n";
        }

        //最大速度
        rc = modbus_write_register(ct, 6148, 0);
        rc = modbus_write_register(ct, 6149, 7000);

        //加速度
        rc = modbus_write_register(ct, 6150, 0);
        rc = modbus_write_register(ct, 6151, 70000);

        //減速度
        rc = modbus_write_register(ct, 6152, 0);
        printf("6152 rc=%d\n",rc);
        rc = modbus_write_register(ct, 6153, 70000);
        printf("6153 rc=%d\n",rc);

        //運轉電流
        rc = modbus_write_register(ct, 6154, 0);
        rc = modbus_write_register(ct, 6155, 500);

        //結合
        rc = modbus_write_register(ct, 6158, 0);
        rc = modbus_write_register(ct, 6159, 0);

        //輸入啟動
        rc = modbus_write_register(ct, 125, 8);

        //輸出結束
        rc = modbus_write_register(ct, 127, 8);

        pub = false;
    }
}

bool Is_LMBusy(modbus_t* ct, uint16_t * tab_rp_registers, uint16_t * tab_rq_registers)
{
    curr_state = modbus_write_register(ct, 257, 1);
    curr_state = modbus_read_registers(ct, 257, 1, tab_rp_registers);
    return (tab_rq_registers[0] != tab_rp_registers[0])?false:true;
}
int Get_CurrPos(modbus_t* ct, uint16_t * tab_rp_registers, uint16_t * tab_rq_registers)
{
    curr_state = modbus_write_register(ct, 291, 1);
    curr_state = modbus_read_registers(ct, 291, 1, tab_rp_registers);
    return tab_rp_registers[0];
}

linear_motion::LM_Cmd Update_LMMsg(uint16_t * tab_rp_registers, uint16_t * tab_rq_registers, std::string &LM_x_state, std::string &LM_z_state)
{
    static bool is_x_busy = false;
    static bool is_z_busy = false;
    linear_motion::LM_Cmd LM;

    is_x_busy = Is_LMBusy(ctx, tab_rp_registers, tab_rq_registers);
    is_z_busy = Is_LMBusy(ctz, tab_rp_registers, tab_rq_registers);

    LM.x_curr_pos = Get_CurrPos(ctx, tab_rp_registers, tab_rq_registers);
    LM.z_curr_pos = Get_CurrPos(ctz, tab_rp_registers, tab_rq_registers);

    if ((LM_x_state == "execute")||(LM_z_state == "execute"))
    {
        if ((is_x_busy == true)||(is_z_busy == true))
            LM.status = "LM_busy";

        else if ((is_x_busy == false)&&(is_z_busy == false))
        {
            LM.status  = "LM_complete";
            LM_x_state = "idle";
            LM_z_state = "idle";
            LM.id = 0;
        }
        else 
        {
            LM.status  = "error";
        }
    }
    else
    {
        LM.status = "LM_idle";
        LM_x_state = "idle";
        LM_z_state = "idle";
    }
    return LM;
}

int main(int argc, char **argv)
{
    bool isbusy   = false;
    int  curr_pos = -1;
    std::string LM_x_state = "idle";
    std::string LM_z_state = "idle";
    // For Allocate_and_Init_MemorySpace
    int nb = 99;
    uint16_t *tab_rq_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    uint16_t *tab_rp_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    memset(tab_rq_registers, 0, nb * sizeof(uint16_t));
    memset(tab_rp_registers, 0, nb * sizeof(uint16_t));

    //========================= Initialize ROS =============================
    ros::init(argc, argv, "linear_z");
    
    ros::NodeHandle nh_param("~");
    std::string port_x;
    std::string port_z;
    int  baud_rate;

    nh_param.param<std::string>("port_x", port_x,"/dev/arc/LM1");
    nh_param.param<std::string>("port_z", port_z,"/dev/arc/LM2");//USB1
    
    nh_param.param<int>("baud", baud_rate, 9600);
    // std::cout<<port<<"\n";

    //========================= Initialize Modbus_RTU ============================= 
    bool Connect_X_OK = false;
    bool Connect_Z_OK = false;
    ctx = Init_Modus_RTU(Connect_X_OK, 1, "/dev/arc/LM1", 9600);
    ctz = Init_Modus_RTU(Connect_Z_OK, 2, "/dev/arc/LM2", 9600);//USB1
    
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
    ros::Publisher  pub = n.advertise<linear_motion::LM_Cmd>("/LM_FeedBack", 1);
    ros::Rate loop_rate(10);

    // ============================= ROS Loop =============================
    bool is_send1 = false;
    while (ros::ok())
    {
        if(LM_Msg.id == 1)  {SendCmd(true, ctx, LM_Msg.x); LM_x_state = "execute";}
        if(LM_Msg.id == 2)  {SendCmd(true, ctz, LM_Msg.z); LM_z_state = "execute";}

        if(LM_Msg.id == 3)  
        {
            
            SendCmd(true, ctz, LM_Msg.z); 
            ros::Duration(0.3);
            SendCmd(true, ctx, LM_Msg.x); 
            LM_x_state = "execute";
            LM_z_state = "execute";
        }
        if(LM_Msg.id == 4)  
        {
            SendCmd(true, ctx, LM_Msg.x); 
            ros::Duration(0.3);
            SendCmd(true, ctz, LM_Msg.z); 
            LM_x_state = "execute";
            LM_z_state = "execute";
        }
        
        // LM_Msg = Update_LMMsg(ctx, tab_rp_registers, tab_rq_registers, LM_state);
        // LM_Msg = Update_LMMsg(ctz, tab_rp_registers, tab_rq_registers, LM_state);

        LM_Msg = Update_LMMsg(tab_rp_registers, tab_rq_registers, LM_x_state, LM_z_state);
        pub.publish(LM_Msg);
        
        // std::cout << "isbusy = " << LM_Msg.isbusy << "\n";
        // std::cout << "curr pos = " << LM_Msg.curr_pos << "\n";
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
