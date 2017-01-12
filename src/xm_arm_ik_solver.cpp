#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <xm_msgs/xm_SolveIK.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

const double max_distence = 1.05;//TODO
const double L0 = 0.16;//大臂电机到arm_base的距离
const double L1 = 0.35;
const double L2 = 0.3;
const double L3 = 0.22;
//爪子长10cm,腕部15cm

const double joint_range_max[5] = {0.98, 1.3, 2.4, 2.0, 1.57};
const double joint_range_min[5] = {-0.82, -1.4, -2.4, -2.0, -1.57};

class xm_arm_ik_solver
{
public:
	xm_arm_ik_solver(ros::NodeHandle &n)
		:nh(n),tf_(ros::Duration(5.0))
	{
		ik_server = nh.advertiseService("xm_ik_solver", &xm_arm_ik_solver::service_callback,this);
	}
	~xm_arm_ik_solver()
    {
    }
private:
	bool service_callback(xm_msgs::xm_SolveIKRequest &req, xm_msgs::xm_SolveIKResponse &res)
	{
		//0.65~1.05
		geometry_msgs::PointStamped goal;
		std::vector<double> value;
		value.resize(5,0.0);
		bool result  = false;
		if(req.goal.header.frame_id !="arm_base")
		{
			tf_.transformPoint("arm_base", req.goal, goal);
		}
		else{
			goal = req.goal;
		}
      		std::cout<<"IK Goal"<<goal.point.x<<" "<<goal.point.y<<" "<<goal.point.z<<std::endl;
		result = compute_ik(goal.point, value);
		if(result)
		{
			res.result = true;
			res.solution = value;
		}
		else{
			res.result = false;
			if(goal.point.x > 0.8)
			{
				res.message ="move_forward";
			}
			if(goal.point.x < 0.65)
			{
				res.message ="move_back";
			}
			if(goal.point.y>0.3)
			{
				res.message  ="move_left";
			}
			if(goal.point.y <-0.3)
			{
				res.message  =" move_right";
			}
		}
		return true;
	}
	bool compute_ik(const geometry_msgs::Point &goal,std::vector<double> &joint_value)
	{
		joint_value.resize(5,0.0);
		double distance = sqrt(goal.x* goal.x + goal.y*goal.y + goal.z*goal.z);//以大臂轴为基准，机械臂所在平面坐标
		std::cout<<"Distance "<<distance<<std::endl;
		if(distance > max_distence )
		{
			ROS_ERROR("Too Far");
			return false;	
		} 
		double temp   = sqrt(goal.x*goal.x + goal.y*goal.y);
		double x = temp - L0 - L3;
		double y = goal.z;
		std::cout<<"x"<<x<<"y"<<y<<std::endl;
		double th1 = atan2(y,x);
		double th2 = acos((x*x +y*y -L1*L1 -L2*L2)/(2*L1*L2));
		double th3 = acos( (x*x+y*y+L1*L1 -L2*L2)/(2*L1*sqrt(x*x+y*y)));

		joint_value[0] = atan2(goal.y,goal.x);
		joint_value[1] = th1 + th3;
		joint_value[2] = -th2;//小臂关节值优先小于0
		joint_value[3] = -(joint_value[1] + joint_value[2]);
		joint_value[4] = 0.0;

		if(check(joint_value))
		{
			ROS_INFO("IK Succeed !");
			return true;
		}
		else{
			joint_value[2] = th2;
			joint_value[1] = th1 -th3;
			if(check(joint_value))
			{
				ROS_INFO("IK Succeed !");
				return true;
			} 
			ROS_INFO("IK Failed !");
			return false;
		}
	}
	bool check(const std::vector<double> value)
	{
		for (int i = 0; i < value.size(); ++i)
		{
			if((value[i] > joint_range_max[i]) || (value[i]<joint_range_min[i])  ||(isnan(value[i]) ) )
			{
				ROS_ERROR("Value Not Fit !");
				std::cout<<"Invalid value: ";
				for (int i = 0; i < value.size(); ++i)
				{
					std::cout<<value[i]<<" ";
				}
				std::cout<<std::endl;
				return false;
			}
		}
		ROS_INFO("check succeed!");
		return true;
	}
	ros::NodeHandle nh;
	ros::ServiceServer  ik_server;
	tf::TransformListener tf_;	
};


int main(int argc, char **argv)
{
	ros::init(argc,argv,"xm_ik_server");
	ros::NodeHandle n;
	xm_arm_ik_solver solver(n);
	ros::spin();
	return 0;
}
