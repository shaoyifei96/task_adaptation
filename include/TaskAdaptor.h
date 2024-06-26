#ifndef TASKADAPTOR_H
#define TASKADAPTOR_H


#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Pose.h"

#include <dynamic_reconfigure/server.h>
#include <task_adaptation/task_adaptation_paramsConfig.h>
//include eigen
#include <Eigen/Dense>


class TaskAdaptor {

private:

	// ros variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	ros::Duration disp_rate_;

	ros::Subscriber sub_realVelocity_;
	ros::Subscriber sub_task1_;
	ros::Subscriber sub_task2_;
	ros::Subscriber sub_task3_;
	ros::Subscriber sub_task4_;
	ros::Subscriber sub_task_state_;

	//for human detection
	ros::Subscriber sub_human_tank_state_;
	ros::Subscriber sub_human_admittance_velocity_;

	ros::Publisher pub_adapted_velocity_;
	ros::Publisher pub_wrench_control_;
	ros::Publisher pub_beliefs_;


	geometry_msgs::TwistStamped  msgAdaptedVelocity_;
	geometry_msgs::WrenchStamped msgWrenchControl_;

	//dynamic reconfig settig
	dynamic_reconfigure::Server<task_adaptation::task_adaptation_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<task_adaptation::task_adaptation_paramsConfig>::CallbackType dyn_rec_f_;

	Eigen::Matrix3d rot_mat_ = Eigen::Matrix3d::Identity();
	Eigen::Vector3d robot_state_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d robot_euler_ = Eigen::Vector3d::Zero();	
	double human_state_0_1_;
	Eigen::VectorXd human_admittance_velocity_ = Eigen::VectorXd::Zero(6);
	

	// topic names
	std::string topic_real_velocity_;
	std::string topic_task1_velocity_;
	std::string topic_task2_velocity_;
	std::string topic_task3_velocity_;
	std::string topic_task4_velocity_;
	std::string topic_adapted_velocity_;
	std::string topic_desired_force_;

	// task adaptation variables
	std::vector<float> RealVelocity_;
	std::vector<float> DesiredVelocity_;
	std::vector<float> DesiredAngVelocity_;
	std::vector<float> ControlWrench_;

	// the null primitive always commands zero velocity
	const std::vector<float> Task0_velocity_ = {0, 0, 0};

	std::vector<float> Task1_velocity_;
	std::vector<float> Task1_ang_velocity_;
	std::vector<float> Task2_velocity_;
	std::vector<float> Task2_ang_velocity_;
	std::vector<float> Task3_velocity_;
	std::vector<float> Task4_velocity_;

	// vectors to contain beliefs and their updates
	std::vector<bool>  flag_newdata_; // 0 for realvelocity and i for task_i
	std::vector<float> Beliefs_;
	std::vector<float> UpdateBeliefsRaw_;
	std::vector<float> UpdateBeliefs_;

	// adaptation gains
	double epsilon_, epsilon_hack_;

	// control gain for a simple velocity controller
	double D_gain_, D_gain_hack_;



public:

	TaskAdaptor(ros::NodeHandle &n,
	            double frequency,
	            std::string topic_real_velocity,
	            std::string topic_task1_velocity,
	            std::string topic_task2_velocity,
	            std::string topic_task3_velocity,
	            std::string topic_task4_velocity,
	            std::string topic_adapted_velocity,
	            std::string topic_desired_force);

	bool Init();

	void Run();


private:

	void InitClassVariables();

	bool InitROS();

	bool CheckNewData();

	void ComputeNewBeliefs();

	void PublishBeliefs();


	void PublishAdaptedVelocity();

	void ComputeDesiredForce();

	void PublishDesiredForce();

	void updateRealVelocity(const geometry_msgs::Twist::ConstPtr& msg);

	void updateRealVelocity_world(const geometry_msgs::Twist::ConstPtr& msg);


	void UpdateTask1(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void UpdateTask2(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void UpdateTask3(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void UpdateTask4(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void updateRobotState(const geometry_msgs::Pose::ConstPtr& msg);
	void updateHumanV(const geometry_msgs::Twist::ConstPtr& msg);
	void updateHumanTankState(const std_msgs::Float32::ConstPtr& msg);


	void DynCallback(task_adaptation::task_adaptation_paramsConfig& config, uint32_t level);
	// void UpdateParamCallback(const task_adaptation::task_adaptation_params::ConstPtr _msg);

	void DisplayInformation();

	void UpdateDesiredVelocity();

	void RawAdaptation();

	float ComputeInnerSimilarity(float b, std::vector<float> RealVelocity, std::vector<float> task_ang_velocity = {0, 0, 0});

	float ComputeOutterSimilarity(std::vector<float> RealVelocity, std::vector<float> task_ang_velocity = {0, 0, 0});

	void WinnerTakeAll();

	//float ComputeTotalInnerSimilarity();
};

#endif // TASKADAPTOR_H
