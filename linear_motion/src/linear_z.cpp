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
bool pub_flag = false;
modbus_t *ctx, *ctz, *ct_left, *tmp_ct;
int curr_state = 10;

void SendCmd(bool Is_Pub, modbus_t* ct, int pos);
std::string LM_x_state = "idle";
std::string LM_z_state = "idle";
std::string LM_left_state = "idle";
bool is_x_busy = false;
bool is_z_busy = false;
bool is_left_busy = false;

void first_topic_callback(const linear_motion::LM_Cmd::ConstPtr &tmp_LM_Msg)
{
    ROS_INFO("position: [%i]", tmp_LM_Msg->x);
    LM_Msg.x = tmp_LM_Msg->x;
    LM_Msg.z = tmp_LM_Msg->z;
    LM_Msg.left = tmp_LM_Msg->left;
    LM_Msg.id = tmp_LM_Msg->id;
    // std::cout<<"-------------------id = "<<LM_Msg.id<<",left move_dis = "<<LM_Msg.left<<"-------------\n";
    // LM_Msg.is_busy = tmp_LM_Msg->isbusy;
    // LM_Msg.curr_pos = tmp_LM_Msg->curr_pos;
    // if((is_x_busy==false)&&(is_z_busy==false)&&(is_left_busy==false))
    {
        std::cout<<"========== send cmd id = "<<LM_Msg.id<<" ========== \n";
        if(LM_Msg.id == 1)  {SendCmd(true, ctx    , LM_Msg.x);      LM_x_state = "execute";}
        if(LM_Msg.id == 2)  {SendCmd(true, ctz    , LM_Msg.z);      LM_z_state = "execute";}
        if(LM_Msg.id == 3)  {SendCmd(true, ct_left, LM_Msg.left);   LM_left_state = "execute";}
    }
    // else
    // {
    //     std::cout<<"LM not idle\n";
    // }
    pub_flag = true;
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
    // if (pub_flag == true)
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
        rc = modbus_write_register(ct, 6149, 10000);

        //加速度
        rc = modbus_write_register(ct, 6150, 0);
        rc = modbus_write_register(ct, 6151, 80000);

        //減速度
        rc = modbus_write_register(ct, 6152, 0);
        printf("6152 rc=%d\n",rc);
        rc = modbus_write_register(ct, 6153, 80000);
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

        pub_flag = false;
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

linear_motion::LM_Cmd Update_LMMsg(uint16_t * tab_rp_registers, uint16_t * tab_rq_registers, std::string &LM_x_state, std::string &LM_z_state, std::string &LM_left_state)
{
    linear_motion::LM_Cmd LM;

    is_x_busy    = Is_LMBusy(ctx    , tab_rp_registers, tab_rq_registers);
    is_z_busy    = Is_LMBusy(ctz    , tab_rp_registers, tab_rq_registers);
    is_left_busy = Is_LMBusy(ct_left, tab_rp_registers, tab_rq_registers);

    LM.x_curr_pos    = Get_CurrPos(ctx    , tab_rp_registers, tab_rq_registers);
    LM.z_curr_pos    = Get_CurrPos(ctz    , tab_rp_registers, tab_rq_registers);
    LM.left_curr_pos = Get_CurrPos(ct_left, tab_rp_registers, tab_rq_registers);  

    // std::cout<< "is_x_busy = " << is_x_busy << ", is_z_busy = " << is_z_busy << ", is_left_busy = " << is_left_busy << std::endl;
    // std::cout<< "LM_x_state = " << LM_x_state << ", LM_z_state = " << LM_z_state << ", LM_left_state = " << LM_left_state << std::endl;

    if ((LM_x_state == "execute")||(LM_z_state == "execute")||(LM_left_state == "execute"))
    {
        if ((is_x_busy == true)||(is_z_busy == true)||(is_left_busy == true))
        {
            // std::cout<<"=== 2 ===\n";
            LM.status = "LM_busy";
            if(is_x_busy == false)         LM_x_state       = "idle";
            if(is_z_busy == false)         LM_z_state       = "idle";
            if(is_left_busy == false)      LM_left_state    = "idle";

        }

        else if ((is_x_busy == false)&&(is_z_busy == false)&&(is_left_busy == false))
        {
            // std::cout<<"=== 3 ===\n";
            LM.status     = "LM_complete";
            LM_x_state    = "idle";
            LM_z_state    = "idle";
            LM_left_state = "idle";
            LM.id = 5;
        }
        else 
        {
            std::cout<<"=== 4 ===\n";
            LM.status  = "error";
        }
    }
    else
    {
        LM.status = "LM_idle";
        LM_x_state = "idle";
        LM_z_state = "idle";
        LM_left_state = "idle";
    }

    return LM;
}

int main(int argc, char **argv)
{
    bool isbusy   = false;
    int  curr_pos = -1;

    // For Allocate_and_Init_MemorySpace
    int nb = 99;
    uint16_t *tab_rq_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    uint16_t *tab_rp_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    memset(tab_rq_registers, 0, nb * sizeof(uint16_t));
    memset(tab_rp_registers, 0, nb * sizeof(uint16_t));

    //========================= Initialize ROS =============================
    ros::init(argc, argv, "linear_z");
    
    ros::NodeHandle nh_param("~");
    std::string port_right;
    std::string port_base;
    std::string port_left;
    int  baud_rate;

    nh_param.param<std::string>("port_right", port_right,"/dev/arc/LM1");
    nh_param.param<std::string>("port_base", port_base,"/dev/arc/LM2");//USB1
    nh_param.param<std::string>("port_left", port_left,"/dev/arc/LM3");//USB3
    
    nh_param.param<int>("baud", baud_rate, 9600);
    // std::cout<<port<<"\n";

    //========================= Initialize Modbus_RTU ============================= 
    bool Connect_X_OK = false;
    bool Connect_Z_OK = false;
    bool Connect_Left_OK = false;
    ctx     = Init_Modus_RTU(Connect_X_OK   , 1, "/dev/arc/LM1", 9600);
    ctz     = Init_Modus_RTU(Connect_Z_OK   , 2, "/dev/arc/LM2", 9600);//USB1
    ct_left = Init_Modus_RTU(Connect_Left_OK, 3, "/dev/arc/LM3", 9600);//USB3
    
    if((Connect_X_OK == false)||(Connect_Z_OK == false)||(Connect_Left_OK == false))
    {
        if(Connect_X_OK    == false)   std::cout<<"CONNECT X ERROR!!!!!\n";
        if(Connect_Z_OK    == false)   std::cout<<"CONNECT Z ERROR!!!!!\n";
        if(Connect_Left_OK == false)   std::cout<<"CONNECT Left ERROR!!!!!\n";
        return -1;
    }
    else
    {
        std::cout<<"All LM Connect ok\n";
    }

    // ============================= Subscribe message =============================
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/position_topic", 10, first_topic_callback);
    ros::Publisher  pub = n.advertise<linear_motion::LM_Cmd>("/LM_FeedBack", 1);
    ros::Rate loop_rate(10);

    // ============================= ROS Loop =============================
    bool is_send1 = false;
    while (ros::ok())
    {
        // //
        // if(LM_Msg.status == "LM_idle")
        // {
        //     if(LM_Msg.id == 1)  {SendCmd(true, ctx    , LM_Msg.x);      LM_x_state = "execute";}
        //     if(LM_Msg.id == 2)  {SendCmd(true, ctz    , LM_Msg.z);      LM_z_state = "execute";}
        //     if(LM_Msg.id == 3)  {SendCmd(true, ct_left, LM_Msg.left);   LM_left_state = "execute";}
        // }
        // else
        // {
        //     std::cout<<"LM not idle\n";
        // }
        
        // LM_Msg = Update_LMMsg(ctx, tab_rp_registers, tab_rq_registers, LM_state);
        // LM_Msg = Update_LMMsg(ctz, tab_rp_registers, tab_rq_registers, LM_state);

        LM_Msg = Update_LMMsg(tab_rp_registers, tab_rq_registers, LM_x_state, LM_z_state, LM_left_state);
        pub.publish(LM_Msg);
        
        // std::cout << "isbusy = " << LM_Msg.isbusy << "\n";
        // std::cout << "curr pos = " << LM_Msg.curr_pos << "\n";
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
