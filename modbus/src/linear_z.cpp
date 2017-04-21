#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include "modbus/modbus.h"

#define LOOP 1
#define SERVER_ID 1
#define ADDRESS_START 0
#define ADDRESS_END 99
#define position

std_msgs::Int32 std_msg;
std_msgs::Int32 feedback;

bool pub = false;

void first_topic_callback(const std_msgs::Int32::ConstPtr &msg)
{
    ROS_INFO("position: [%i]", msg->data);
    std_msg.data = msg->data;
    pub = true;
}

int main(int argc, char **argv)
{
    modbus_t *ctx;
    int rc;
    int nb_fail;
    int nb_loop;
    int addr;
    int nb;
    int i;
    uint8_t *tab_rq_bits;
    uint8_t *tab_rp_bits;
    uint16_t *tab_rq_registers;
    uint16_t *tab_rw_rq_registers;
    uint16_t *tab_rp_registers;

    /* RTU */

    ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'E', 8, 1);
    modbus_set_slave(ctx, SERVER_ID);
    if (modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n",
                modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    /* Allocate and initialize the different memory spaces */
    nb = ADDRESS_END - ADDRESS_START;

    tab_rq_bits = (uint8_t *)malloc(nb * sizeof(uint8_t));
    memset(tab_rq_bits, 0, nb * sizeof(uint8_t));

    tab_rp_bits = (uint8_t *)malloc(nb * sizeof(uint8_t));
    memset(tab_rp_bits, 0, nb * sizeof(uint8_t));

    tab_rq_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    memset(tab_rq_registers, 0, nb * sizeof(uint16_t));

    tab_rp_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    memset(tab_rp_registers, 0, nb * sizeof(uint16_t));

    tab_rw_rq_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    memset(tab_rw_rq_registers, 0, nb * sizeof(uint16_t));

    ros::init(argc, argv, "linear_z");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("position_topic", 10, first_topic_callback);
    ros::NodeHandle n2;
    ros::Publisher feedback_pub = n2.advertise<std_msgs::Int32>("feedback", 10);
    ros::Rate loop_rate(10);

    //==========NO0============//

    while (ros::ok())
    {
        //feedback
        rc = modbus_write_register(ctx, 127, 0);
        printf("127 rc=%d\n", rc);

        rc = modbus_write_register(ctx, 291, 1);
        printf("291 rc=%d\n", rc);
        rc = modbus_read_registers(ctx, 291, 1, tab_rp_registers);
        if (rc != 1)
        {
            printf("ERROR modbus_read_registers single (%d)\n", rc);
            printf("Address = %d\n", addr);
            nb_fail++;
        }
        else
        {
            if (tab_rq_registers[0] != tab_rp_registers[0])
            {
                printf("ERROR modbus_read_registers single\n");
                printf("Address = %d, value = %d (0x%X) != %d (0x%X)\n",
                       addr, tab_rq_registers[0], tab_rq_registers[0],
                       tab_rp_registers[0], tab_rp_registers[0]);
                nb_fail++;
            }
        }

        //////////////////
        feedback.data = *tab_rp_registers;
        feedback_pub.publish(feedback);
        ROS_INFO("feedback_pub = %i", feedback.data);
        if (pub == true)
        {
            //輸入寫入
            rc = modbus_write_register(ctx, 125, 0);
            printf("125 rc=%d\n", rc);
            //運轉方式
            rc = modbus_write_register(ctx, 6144, 0);
            printf("6144 rc=%d\n", rc);
            rc = modbus_write_register(ctx, 6145, 1);
            printf("6145 rc=%d\n", rc);
            //位置
            rc = modbus_write_register(ctx, 6146, 0);
            printf("6146 rc=%d\n", rc);
            rc = modbus_write_register(ctx, 6147, std_msg.data);
            printf("6147 rc=%d\n", rc);
            //速度
            rc = modbus_write_register(ctx, 6148, 0);
            printf("6148 rc=%d\n", rc);
            rc = modbus_write_register(ctx, 6149, 1000);
            printf("6149 rc=%d\n", rc);
            //起動
            rc = modbus_write_register(ctx, 6150, 0);
            printf("6150 rc=%d\n", rc);
            rc = modbus_write_register(ctx, 6151, 700000);
            printf("6151 rc=%d\n", rc);
            //停止
            //rc = modbus_write_register(ctx, 6152, 0);
            //printf("6152 rc=%d\n",rc);
            //rc = modbus_write_register(ctx, 6153, 700000);
            //printf("6153 rc=%d\n",rc);
            //運轉電流
            rc = modbus_write_register(ctx, 6154, 0);
            printf("6154 rc=%d\n", rc);
            rc = modbus_write_register(ctx, 6155, 500);
            printf("6155 rc=%d\n", rc);
            //結合
            rc = modbus_write_register(ctx, 6158, 0);
            printf("6158 rc=%d\n", rc);
            rc = modbus_write_register(ctx, 6159, 0);
            printf("6159 rc=%d\n", rc);
            //輸入啟動
            rc = modbus_write_register(ctx, 125, 8);
            printf("125 rc=%d\n", rc);
            //輸出結束
            rc = modbus_write_register(ctx, 127, 8);

            loop_rate.sleep();
            pub = false;
        }
        ros::spinOnce();
    }
    return 0;
}
