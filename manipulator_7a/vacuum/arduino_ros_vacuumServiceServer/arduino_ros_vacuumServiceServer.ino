/*
   rosserial Service Server for Vacuum
*/

#include <ros.h>
#include <vacuum_cmd_msg/VacuumCmd.h>

#define CTRL 12
#define LED  13

ros::NodeHandle  nh;
using vacuum_cmd_msg::VacuumCmd;

void ctrl_Output(bool);
void callback(const VacuumCmd::Request& , VacuumCmd::Response& );

ros::ServiceServer<VacuumCmd::Request, VacuumCmd::Response> vac_srv("robot_cmd", &callback);

void setup()
{
  nh.initNode();
  nh.advertiseService(vac_srv);

  pinMode(CTRL, OUTPUT);
  pinMode(LED,  OUTPUT);
  ctrl_Output(false);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}

void ctrl_Output(bool state)
{
  digitalWrite(CTRL, state);
  digitalWrite(LED,  state);
}

void callback(const VacuumCmd::Request& req, VacuumCmd::Response& res)
{
  if (strcmp(req.cmd, "vacuumOn") == 0)
  {
    ctrl_Output(true);
  }
  else if (strcmp(req.cmd, "vacuumOff") == 0)
  {
    ctrl_Output(false);
  }
  else
  {
    res.success = false;
    return;
  }
  res.success = true;
}
