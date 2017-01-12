#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <xm_msgs/xm_SolveIK.h>
#include <xm_msgs/xm_PickOrPlace.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <xm_msgs/xm_Gripper.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

const double prepare_value[5]  ={1.0, 1.4, -2.5, 1.1, 0.0};
const double nav_value[5] ={0.0, 1.5, -2.66, -2.0,0.0};
class xm_pick_or_place
{
public:
	xm_pick_or_place(ros::NodeHandle &n)
		:nh(n)
	{
		xm_gripper_client = nh.serviceClient<xm_msgs::xm_Gripper>("xm_robot/gripper_command");
		xm_ik_client = nh.serviceClient<xm_msgs::xm_SolveIK>("xm_ik_solver");
		arm_server = nh.advertiseService("xm_pick_or_place", &xm_pick_or_place::service_callback,this);
		clientptr  = boost::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> >("/xm_robot/xm_arm_controller/follow_joint_trajectory",true);
		clientptr->waitForServer(ros::Duration(5.0));
     		init_value.push_back(1.0);
        		init_value.push_back(1.4);
        		init_value.push_back(-2.5);
        		init_value.push_back(1.1);
        		init_value.push_back(0.0);
		ROS_INFO("Connect Succeed !");
	}
	~xm_pick_or_place()
    {
    }
private:
	bool service_callback(xm_msgs::xm_PickOrPlaceRequest &req,xm_msgs::xm_PickOrPlaceResponse &res)
	{
		xm_msgs::xm_SolveIK srv;
      		if(req.action ==0)
        		{
            			res.result=init();
        		}
		if(req.action==1)
		{
			srv.request.goal = req.goal_position;
			srv.request.goal.point.x  += 0.05;
			srv.request.goal.point.z   += 0.02;
			srv.request.goal.header.stamp = ros::Time::now()-ros::Duration(1.0);
			xm_ik_client.call(srv);
			if(!srv.response.result)
			{
				ROS_ERROR("IK failed!");//TODO add feedback
				res.result = false;
				return true;
			}
	            		for(int i=0 ; i<5 ;i++) std::cout<<srv.response.solution[i]<<" ";
	            		std::cout<<std::endl;
			if(srv.response.result)
			{
				res.result = pick(srv.response.solution);
			}
			else{
				res.result = false;
				ROS_INFO("%s",srv.response.message.c_str());
			}
		}
		if(req.action ==2)
		{
			srv.request.goal = req.goal_position;
			srv.request.goal.header.stamp = ros::Time::now()-ros::Duration(1.0);
			xm_ik_client.call(srv);
			if(srv.response.result)
			{
				res.result = place(srv.response.solution);
			}
			else{
				res.result = false;
				ROS_INFO("%s",srv.response.message.c_str());
			}
		}
		if(req.action ==3)
		{
			res.result = prepare_nav();
			ROS_INFO("Prepare to navigation!");
		}
		return true;
	}
	bool prepare_nav()
	{
		std::vector<double> temp(nav_value,nav_value+5);
		if(!execute(temp)) return false;
		return true;
	}
	bool pick(std::vector<double> &goal)
	{
		ROS_INFO("pick");
		std::vector<double> temp(init_value);
		temp[0] = goal[0];
		if(!execute(temp) ) return false;
		ROS_INFO("pick_turn!");
		temp = goal;
		temp[1]+=0.13;
		temp[2]-= 0.13;
		if(!execute(temp) ) return false;
		if(!execute(goal) ) return false;
		ros::Duration(2.0).sleep();
		if(!gripper_exe(1)) return false;
		ROS_INFO("I pick it!");
		simple_move_x(1,goal);
		//if(!execute(goal) ) return false;
		//ros::Duration(3.0).sleep();
		if(!init(goal)) return false;
       		if(!init()) return false;
		return true;
	}
	bool place(std::vector<double> &goal)
	{
		ROS_INFO("place");
		std::vector<double> temp(init_value);
		temp[0] = goal[0];
		if(!execute(temp) ) return false;
		ROS_INFO("place turn!");
		simple_move_x(1,goal);
		if(!execute(goal) ) return false;
		ros::Duration(2.0).sleep();
		if(!gripper_exe(0)) return false;
		ros::Duration(1.5).sleep();
		ROS_INFO("I place it!");
		if(!init(goal)) return false;
     		if(!init()) return false;
		return true;
	}
	bool init ()
	{
		std::vector<double> temp_value(5,0.0);
		temp_value[0] = 1.0;
		temp_value[1] = 1.4;
		temp_value[2] = -2.5;
		temp_value[3] = 1.1;
		temp_value[4] = 0.0;
		if(!execute(temp_value)) return false;
		ROS_INFO("Init State!");
       		 return true;
	}
	bool init(const std::vector<double> &goal)
	{
		std::vector<double> temp_value(5,0.0);
		temp_value[0] = goal[0];
		temp_value[1] = 1.4;
		temp_value[2] = -2.5;
		temp_value[3] = 1.1;
		temp_value[4] = 0.0;
		if(!execute(temp_value)) return false;
		ROS_INFO("init turn");
		temp_value[0] = 1.0;
		if(!execute(temp_value)) return false;
		ROS_INFO("Init State!");
       		 return true;
	}
	bool execute(const std::vector<double> &goal)
	{
		if(goal.size()!=5)
		{
			ROS_ERROR("Can not execute the num of goal is not 5");
			return false;
		}
       		for(int i=0 ; i<5 ;i++) std::cout<<"Execute"<<goal[i]<<" ";
    		std::cout<<std::endl;
		control_msgs::FollowJointTrajectoryGoal fjt;
		fjt.trajectory.joint_names.push_back("joint_waist");
		fjt.trajectory.joint_names.push_back("joint_big_arm");
		fjt.trajectory.joint_names.push_back("joint_forearm");
		fjt.trajectory.joint_names.push_back("joint_wrist_pitching");
		fjt.trajectory.joint_names.push_back("joint_wrist_rotation");
		fjt.trajectory.points.resize(1);
		fjt.trajectory.points.at(0).positions = goal;
		fjt.trajectory.points.at(0).accelerations.resize(5, 0.0);
		fjt.trajectory.points.at(0).velocities.resize(5, 0.0);
		fjt.trajectory.points.at(0).effort.resize(5, 0.0);
		fjt.trajectory.points.at(0).time_from_start = ros::Duration(2.0);
		fjt.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
		clientptr->sendGoal(fjt);
		ROS_INFO("Send new trajectory!");
	    	clientptr->waitForResult(ros::Duration(10.0));
	    	if(clientptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	   	{
	   		ROS_INFO("SUCCEEDED");
		    	return true;
		}
		else {
	              	ROS_ERROR("execute failed!");
		    	return false;
		}
	}
	
	bool gripper_exe(int mod)
	{
		xm_msgs::xm_Gripper srv;
		if(mod ==1)
		{
			srv.request.command  = true;
		}
		if(mod ==0)
		{
			srv.request.command = false;
		}
		xm_gripper_client.call(srv);
		while(! srv.response.result);
		xm_gripper_client.call(srv);
		std::cout<<srv.response.message<<std::endl;
		std::cout<<"Gripper Succeed!"<<std::endl;
		return srv.response.result;//服务是否阻塞待定
	}
	bool simple_move_x(int mod,const std::vector<double> &goal)
	{
		std::vector<double> temp;
		temp  = goal;
		if(mod==0)
		{
		     temp[1]-=0.4;
		     temp[2]+=0.4;
		}
		if(mod ==1)
		{
			temp[1]+=0.25;
			temp[2]-=0.25;
			temp[3]  += 1.0;
		}
		if(execute(temp))  return true;
		return false;
	}
	
	ros::NodeHandle nh;
   	std::vector<double> init_value;
   	std::vector<double> current_cmd;
	boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> >  clientptr;
	ros::ServiceClient xm_ik_client;
	ros::ServiceClient xm_gripper_client;
	ros::ServiceServer arm_server;
};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"arm_server");
    ros::AsyncSpinner spiner(4);
    spiner.start();
    ros::NodeHandle n;
    xm_pick_or_place server(n);
    while(ros::ok());
    return 0;
}
