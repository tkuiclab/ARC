
ROS_INFO("(KBE) In ");

~manipulator_h_manager package~
/******main cpp******/
##manipulator_h_manager.cpp
  controller->addMotionModule((robotis_framework::MotionModule*) BaseModule::getInstance());
  controller->startTimer();


/******controller->addMotionModule()******/
RobotisController::addMotionModule() 
	module->initialize(CONTROL_CYCLE_MSEC, robot_);
		BaseModule::initialize()
			&BaseModule::queueThread()
	motion_modules_.push_back(module);

/******controller->startTimer();*******/
RobotisController::startTimer()
	en_sim = true -> gazeboTimerThread()   /* Loop gazeboTimerThread()*/
	en_sim = false -> timerThread()	      /* Loop timerThread()*/
	this->is_timer_running_ = true;



/******Subscriber & Publisher******/
~manipulator_h_manager.cpp~
controller->initialize()
RobotisController::initialize()
	&RobotisController::msgQueueThread    (KBE: NOT LOOP)
		/*Subscriber & Publisher*/
		ros::Subscriber sync_write_item_sub     = ros_node.subscribe("/robotis/sync_write_item", 10,
                                                               &RobotisController::syncWriteItemCallback, this);
		 goal_joint_state_pub_     = ros_node.advertise<sensor_msgs::JointState>("/robotis/goal_joint_states", 10);
 		
		....
		....etc.



/******RobotisController::gazeboTimerThread()******/
 while (!stop_timer_)
  {
    if (init_pose_loaded_ == true)
      process();
    gazebo_rate.sleep();
  }
	

#init_pose_loaded_ = true;
1. en_sim 
	 RobotisController::gazeboJointStatesCallback()
2. !en_sim
	RobotisController::initializeSyncWrite()	



/******RobotisController::process()******/
1. en_sim(Gazebo) 
	sensor_modules_.size() = 0
	if (sensor_modules_.size() > 0){
		ignore...
	}	

	//Loop this
	if (controller_mode_ == MotionModuleMode){
		//(KBE_Note) Click [Set Mode] In UI will let getModuleEnable() be True
		module_it = base_module
		if ((*module_it)->getModuleEnable() == false)
         		continue;
		/******After getModuleEnable() == True******/
		(*module_it)->process(robot_->dxls_, sensor_result_);
		
		//for (joint_name=  joint1 ->  joint6  ){
		for (auto& dxl_it : robot_->dxls_){
			std::string     joint_name  = dxl_it.first;
		  	Dynamixel      *dxl         = dxl_it.second;
		  	DynamixelState *dxl_state   = dxl_it.second->dxl_state_;
			//(KBE_Note) dxl->ctrl_module_name_ = base_module
         		if (dxl->ctrl_module_name_ == (*module_it)->getModuleName()){
				
				//(KBE_Note) result_state->present_position_ = 0 always??
             			//(KBE_Note) result_state->goal_position_ =  0 -> 1.5 increasely, if target = 1.5
				DynamixelState *result_state = (*module_it)->result_[joint_name];
 
				//(KBE_Note) Default is PositionControl
				if ((*module_it)->getControlMode() == PositionControl){
					//(KBE_Note) dxl_state->present_position_ = 0 -> 1.5 increasely, try to pursuit 
             				//(KBE_Note) dxl_state->goal_position_ =  0 -> 1.5 increasely, if target = 1.5
					dxl_state->goal_position_ = result_state->goal_position_;


				}else if ((*module_it)->getControlMode() == VelocityControl){
				}else if ((*module_it)->getControlMode() == TorqueControl){
				}

			}

		}		
	}

	 // SyncWrite
    if (gazebo_mode_ == false && do_sync_write)
    {
	/* WRITE COMMAND TO DIVICE!!!*/
    } else if (gazebo_mode_ == true){
	for (auto& dxl_it : robot_->dxls_)
        {
          std::string     joint_name  = dxl_it.first;
          Dynamixel      *dxl         = dxl_it.second;
          DynamixelState *dxl_state   = dxl_it.second->dxl_state_;

          if (dxl->ctrl_module_name_ == (*module_it)->getModuleName())
          {
            if ((*module_it)->getControlMode() == PositionControl)
            {
	      //(KBE_NOTE) /robotis_manipulator_h/joint1_position/command
              joint_msg.data = dxl_state->goal_position_;
              gazebo_joint_position_pub_[joint_name].publish(joint_msg);
	     
            }
	  }
	}
    }
    // publish present & goal position
    for (auto& dxl_it : robot_->dxls_){
		std::string joint_name  = dxl_it.first;
		Dynamixel  *dxl         = dxl_it.second;

		present_state.name.push_back(joint_name);
		present_state.position.push_back(dxl->dxl_state_->present_position_);
		present_state.velocity.push_back(dxl->dxl_state_->present_velocity_);
		present_state.effort.push_back(dxl->dxl_state_->present_torque_);

		goal_state.name.push_back(joint_name);
		goal_state.position.push_back(dxl->dxl_state_->goal_position_);
		goal_state.velocity.push_back(dxl->dxl_state_->goal_velocity_);
		goal_state.effort.push_back(dxl->dxl_state_->goal_torque_);
    }

    //(KBE_NOTE)  present_joint_state_pub_ "/robotis/present_joint_states"
    //(KBE_NOTE)  goal_joint_state_pub_ "/robotis/goal_joint_states"

    // -> publish present joint_states & goal joint states topic
    present_joint_state_pub_.publish(present_state);
    goal_joint_state_pub_.publish(goal_state);



/*****Diretly Control Joint Position In Gazebo******/
NOTE: Turn off the manipulator_h_manager node

rostopic pub /robotis_manipulator_h/joint1_position/command std_msgs/Float64 "data: 0"



/*************************************Base_Module***************************************/


/*****MotionModule::setModuleEnable(bool),_enable = true ******/
//------base_module.cpp-------//
ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/base/set_mode_msg", 5,
                                                        &BaseModule::setModeMsgCallback, this);

void BaseModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  std_msgs::String str_msg;
  str_msg.data = "base_module";

  set_ctrl_module_pub_.publish(str_msg);

  return;
}
set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);


//------robotics_controller.cpp-------//
ros::Subscriber enable_ctrl_module_sub  = ros_node.subscribe("/robotis/enable_ctrl_module", 10,
                                                               &RobotisController::setCtrlModuleCallback, this);


void RobotisController::setCtrlModuleCallback(const std_msgs::String::ConstPtr &msg)
{
  std::string _module_name_to_set = msg->data;

  set_module_thread_ = boost::thread(boost::bind(&RobotisController::setCtrlModuleThread, this, _module_name_to_set));
}

RobotisController::setCtrlModuleThread(std::string ctrl_module)
	(*m_it)->setModuleEnable(true);
		MotionModule::setModuleEnable(true)
			this->enable_ = true;


/******robotis_->is_moving_=true******/
base_module.cpp 
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/base/kinematics_pose_msg", 5,
                                                               &BaseModule::kinematicsPoseMsgCallback, this);

1. /robotis/base/ini_pose_msg Topic Invoke
  Refer: --->>>    /******Click [Go to Initial Pose] Button******/

2. /robotis/base/kinematics_pose_msg Topic Invoke
  Refer: --->>>    //******Click [Send Des Pos.] Button******/

3./robotis/base/joint_pose_msg



/*other about robotis_->is_moving_()*/
bool BaseModule::isRunning()
{
  return robotis_->is_moving_;
}




/*******Node [/manipulator_h_gui]******/
$ rosnode info manipulator_h_gui
Node [/manipulator_h_gui]
Publications: 
 * /robotis/base/joint_pose_msg [manipulator_h_base_module_msgs/JointPose]
 * /robotis/base/set_mode_msg [std_msgs/String]
 * /rosout [rosgraph_msgs/Log]
 * /robotis/base/ini_pose_msg [std_msgs/String]
 * /robotis/base/kinematics_pose_msg [manipulator_h_base_module_msgs/KinematicsPose]



/******Click [Set Mode] Button******/
$ rostopic echo /robotis/base/set_mode_msg
data: set_mode


/******Click [Send Des Joint Val.] Button******/
$ rostopic echo /robotis/base/joint_pose_msg
name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
value: [0.5235987755982988, 0.0, 0.0, 0.0, 0.0, 0.0]


//base_module.cpp
ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/robotis/base/joint_pose_msg", 5,
                                                          &BaseModule::jointPoseMsgCallback, this);
  
BaseModule::jointPoseMsgCallback()
	robotis_->joint_pose_msg_ = *msg;

	tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateJointTrajProcess, this));


void BaseModule::generateJointTrajProcess()
	double tol = 35 * DEGREE2RADIAN; // rad per sec
  	double mov_time = 2.0;
 	//(KBE_NOTE)  Find max_diff  [target_joint_position - now_joint_position]
	robotis_->mov_time_ = max_diff / tol;   //if > 2.0
	robotis_->all_time_steps_ = int(robotis_->mov_time_ / robotis_->smp_time_) + 1;

	robotis_->calc_joint_tra_.resize(robotis_->all_time_steps_, MAX_JOINT_ID + 1);
	
	/*(KBE_NOTE)  
		example: joint1 want 60(1.0472) -> 0(0)
		robotis_->mov_time_ = 2.0
		obotis_->smp_time_ = 0.008
		robotis_->all_time_steps_  = 251 (2 / 0.008 + 1)
		
		calc_joint_tra_ -> 251 * 6 matrix   ( NOTE: Only  column 1 has value)
	*/
	robotis_->cnt_ = 0;
  	robotis_->is_moving_ = true;



/******Click [Go to Initial Pose] Button******/
$ rostopic echo /robotis/base/ini_pose_msg
data: ini_pose
---


//base_module.cpp
ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/base/ini_pose_msg", 5,
                                                        &BaseModule::initPoseMsgCallback, this);

#void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg){
  if (robotis_->is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // parse initial pose
      std::string ini_pose_path = ros::package::getPath("manipulator_h_base_module") + "/config/ini_pose.yaml";
      parseIniPoseData(ini_pose_path);

      tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateInitPoseTrajProcess, this));
      delete tra_gene_thread_;
    }
  }

#BaseModule::generateInitPoseTrajProcess()
	robotis_->calc_joint_tra_.block(0, id, robotis_->all_time_steps_, 1) = tra;
	robotis_->cnt_ = 0;
  	robotis_->is_moving_ = true;


/******Click [Send Des Pos.] Button******/
$ rostopic echo /robotis/base/kinematics_pose_msg
name: arm
pose: 
  position: 
    x: 0.476
    y: 0.211
    z: 0.453
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.258819045103
    w: 0.965925826289
---
ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/base/kinematics_pose_msg", 5,
                                                               &BaseModule::kinematicsPoseMsgCallback, this);


#void BaseModule::kinematicsPoseMsgCallback(const manipulator_h_base_module_msgs::KinematicsPose::ConstPtr& msg){
  if (enable_ == false)
    return;

  robotis_->kinematics_pose_msg_ = *msg;

  robotis_->ik_id_start_ = 0;
  robotis_->ik_id_end_   = END_LINK;

  if (robotis_->is_moving_ == false)
  {
    tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateTaskTrajProcess, this));
    delete tra_gene_thread_;
  }
}

##BaseModule::generateTaskTrajProcess()	
	double tol = 0.1; // m per sec
	double mov_time = 2.0;

	double diff = sqrt( x^2 + y^2 + z^2);
	
	robotis_->mov_time_ = diff / tol;  //if > 2.0
	robotis_->all_time_steps_ = int(robotis_->mov_time_ / robotis_->smp_time_) + 1;
	robotis_->calc_task_tra_.resize(robotis_->all_time_steps_, 3);


	robotis_->cnt_ = 0;
	robotis_->is_moving_ = true;
	robotis_->ik_solve_ = true;

	/*(KBE_NOTE)  
		example: (0.291, 0, 0.558)  want (0.491, 0, 0.558)
		robotis_->mov_time_ = 2.0
		obotis_->smp_time_ = 0.008
		robotis_->all_time_steps_  = 251 (2 / 0.008 + 1)
		
		calc_joint_tra_ -> 251 * 3 matrix   ( NOTE: Only  column 1 has value)
		(0.290752, 0, 0.558)
		...
		...
		(0.37444 , 0, 0.558)
		...
		...
		( 0.491  , 0, 0.558)

	*/




/******BaseModule::process()******/
if (enable_ == false)
	return;


for(joint1->joint6){
	joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
	joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
}

for (int id = 1; id <= MAX_JOINT_ID; id++)
	manipulator_->manipulator_link_data_[id]->joint_angle_ = joint_state_->goal_joint_state_[id].position_;

manipulator_->forwardKinematics(0);

if (robotis_->is_moving_ == true)
	if (robotis_->cnt_ == 0)
		robotis_->ik_start_rotation_ = manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->orientation_;

    	if (robotis_->ik_solve_ == true){
		//-----IK for kinematics_pose_msg Topic Request------//
      		robotis_->setInverseKinematics(robotis_->cnt_, robotis_->ik_start_rotation_);
		std::cout << "(KBE) ik_target_position_=" << robotis_->ik_target_position_ << std::endl;

		ik_target_position_=0.376553
		0.211089
		0.451493

		
	}else{
		for (int id = 1; id <= MAX_JOINT_ID; id++)
        		joint_state_->goal_joint_state_[id].position_ = robotis_->calc_joint_tra_(robotis_->cnt_, id);
    	}
	robotis_->cnt_++;
	
for(joint1->joint6)
	std::string joint_name = state_iter->first;
	result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;


if (robotis_->cnt_ >= robotis_->all_time_steps_ && robotis_->is_moving_ == true)
	ROS_INFO("[end] send trajectory");
	publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

	robotis_->is_moving_ = false;
	robotis_->ik_solve_ = false;
	robotis_->cnt_ = 0;







    ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/robotis/base/joint_pose_msg", 5,
                                                          &BaseModule::jointPoseMsgCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/base/kinematics_pose_msg", 5,
                                                               &BaseModule::kinematicsPoseMsgCallback, this);


controller   
--dynamixel含device
--BaseModule   軌跡
--運動學在       manipulator_h_kinematics_dynamics



final to device is motor angle

_enable -> false 應該可以停下來  因為每次都動很小

status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
規跡開始結束, ik error
  
  
  
   int     max_iter    = 30;   exceed -> error
   
   
   Jacobian -> 微分運動學
   
   一直迭代  到max_iter
   


不同執行序同時再跑
一個產生軌跡
一個產生馬達角度   儲存起來
一個抓出儲存起來的馬達角度 丟給device



manipulator_h_manager

幾顆馬達設定
1. ROBOTIS_MANIPULATOR_H.robot   幾顆馬達設定
2. manipulator_h_kinematics_dynamics
--manipulator_h_kinematics_dynamics_define 



***** 軌跡規劃 *****
void BaseModule::generateInitPoseTrajProcess()
void BaseModule::generateJointTrajProcess()
--規劃關節空間的軌跡
--感覺是 P to P

void BaseModule::generateTaskTrajProcess()
--規劃工作空間的軌跡
--感覺是 Linear

我大多用 kinematics_pose_msg 下命令給手臂
手臂動作看起來像是 linear
下 InitPose 命令
手臂動作看起來像是 p to p
所以我誤以為 Robotis 沒把 p to p and linear 分開做 
gazebo_joint_position_pub_
