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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linear_z_feedback");
    ros::NodeHandle n;
    ros::Publisher my_pub = n.advertise<std_msgs::Int32>("position_topic", 10);
    ros::Rate loop_rate(1);

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

    while (ros::ok())
    {
        //==========NO0============//

        //啟動
        rc = modbus_write_register(ctx, 127, 8);
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

        rc = modbus_write_register(ctx, 127, 0);
        printf("127 rc=%d\n", rc);
        std_msgs::Int32 std_msg;
        std_msg.data = *tab_rp_registers;
        ROS_INFO("%i", std_msg.data);
        my_pub.publish(std_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
