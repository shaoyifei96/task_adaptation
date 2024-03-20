#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"


// from visualization_msgs.msg import MarkerArray, Marker
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

//use Eigen
#include <Eigen/Dense>

// in this file 3d is x,y,roll
Eigen::Vector3d getTaskVelocity(const Eigen::Vector3d& current_position, const Eigen::Vector3d & attractor_position, const Eigen::Vector3d & dynamic_mat_A) {

	Eigen::Vector3d linear_velocity;
	//multiply by element wise
	linear_velocity = dynamic_mat_A.cwiseProduct(current_position - attractor_position);

	return 	linear_velocity;
}


Eigen::Vector3d robot_state = Eigen::Vector3d(0,0,0);
Eigen::Vector3d robot_vel = Eigen::Vector3d(0,0,0);
Eigen::Vector3d robot_goal = Eigen::Vector3d(0.8,0,0);
Eigen::Quaterniond orientation(1,0,0,0);

Eigen::Quaterniond orientation_des(1,0,0,0);

void updateTaskState(const geometry_msgs::Pose::ConstPtr& msg) {
	Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	orientation = q;
	auto euler = q.toRotationMatrix().eulerAngles(1, 0, 2); // roll pitch yaw
	//ros info stream throttle
	// ROS_INFO_STREAM_THROTTLE(1, "euler: " << euler(0) << ", " << euler(1) << ", " << euler(2));
	
	robot_state(0) = msg->position.x;
	robot_state(1) = msg->position.y;
	robot_state(2) = euler(2);

}
void updateGoal(const visualization_msgs::MarkerArray::ConstPtr& msg) {
	robot_goal(0) = msg->markers[0].pose.position.x;
	robot_goal(1) = msg->markers[0].pose.position.y;
	robot_goal(2) = msg->markers[0].pose.position.z;
}
Eigen::Vector3d getTaskVelocity_Ang(const Eigen::Vector3d & attractor, const Eigen::Matrix3d & dynamic_mat_A, Eigen::Quaterniond & orientation_des_local) {
	// Eigen::Vector4d ee_des_quat;

	Eigen::Quaterniond q_pitch(Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitY()));
	//get atan of the attractor[1] and attractor[0]
	Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(-atan2(attractor[1], attractor[0]), Eigen::Vector3d::UnitX()));
	Eigen::Quaterniond q_roll(Eigen::AngleAxisd(attractor[2], Eigen::Vector3d::UnitZ()));

	orientation_des_local = q_roll * q_yaw * q_pitch;
	// ee_des_quat << _robot.ee_des_quat[1], _robot.ee_des_quat[2], _robot.ee_des_quat[3], _robot.ee_des_quat[0];


	// Eigen::Quaterniond orientation_des(ee_des_quat);
	double s1 = orientation.w();
	double s2 = orientation_des_local.w();
	Eigen::Vector3d u1 = orientation.vec();
	Eigen::Vector3d u2 = orientation_des_local.vec();
	Eigen::Matrix3d Su1;
	Su1 << 0.0, -u1(2), u1(1),
	u1(2), 0.0, -u1(0),
	-u1(1), u1(0), 0.0;
	Eigen::Vector3d temp = -s1 * u2 + s2 * u1 - Su1 * u2;
	double u1u2dot = u1.dot(u2);
	Eigen::Vector4d dori;
	dori << temp, s1 * s2 + u1u2dot;
	Eigen::Quaterniond quat_dot(dori);
	Eigen::Vector3d logdq;
	logdq << acos(quat_dot.w()) * (quat_dot.vec() /  quat_dot.vec().norm());

	return dynamic_mat_A * logdq; // todo make this 3x3
}

void updateAngVel(const geometry_msgs::Twist::ConstPtr& msg) {
	robot_vel(2) = msg->angular.z;
}
void updateLinVel(const geometry_msgs::Pose::ConstPtr& msg) {
	double alpha = 0.2;
	robot_vel(0) = alpha*msg->position.x + (1-alpha)*robot_vel(0);
	robot_vel(1) = alpha*msg->position.y + (1-alpha)*robot_vel(1);
}
int main(int argc, char **argv)
{

	Eigen::Vector3d task1_attractor = Eigen::Vector3d(0.80, 0, 0);
	Eigen::Vector3d task2_attractor = Eigen::Vector3d(robot_goal[0], robot_goal[1], robot_goal[2]);
	Eigen::Vector3d task3_attractor = Eigen::Vector3d(robot_goal[0], robot_goal[1], 0.0); 
	// Eigen::Vector3d task3_attractor = Eigen::Vector3d(0.7,0.7,-1.5); //1.0 rad = 57.2958 deg

	// variables
	geometry_msgs::TwistStamped Task1_velocity;
	geometry_msgs::TwistStamped Task2_velocity;
	geometry_msgs::TwistStamped Task3_velocity;
	geometry_msgs::TwistStamped Task4_velocity;
	geometry_msgs::Twist Real_velocity;




	ros::init(argc, argv, "UnitaryTasks");
	ros::NodeHandle n;

	ros::Publisher pub_task1 = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/task1/DesiredVelocity", 1000);
	ros::Publisher pub_task2 = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/task2/DesiredVelocity", 1000);
	ros::Publisher pub_task3 = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/task3/DesiredVelocity", 1000);
	ros::Publisher pub_task4 = n.advertise<geometry_msgs::TwistStamped>("/testUnitary/task4/DesiredVelocity", 1000);
    ros::Subscriber task_state_sub = n.subscribe<geometry_msgs::Pose>("/iiwa/task_states", 1, &updateTaskState, ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    ros::Subscriber goal_sub = n.subscribe<visualization_msgs::MarkerArray>("/start_and_goal", 1, &updateGoal, ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
	//rospy.Publisher('/start_and_goal', MarkerArray, queue_size=10, latch=True)
	
	ros::Subscriber ee_ang_vel_sub = n.subscribe<geometry_msgs::Twist>("/iiwa/ang_vel_ee_frame", 1, &updateAngVel, ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
	//self.ang_vel_ee_frame_pub = rospy.Publisher("/iiwa/ang_vel_ee_frame", Twist, queue_size=10)
	ros::Subscriber ee_lin_vel_sub = n.subscribe<geometry_msgs::Pose>("/iiwa/ee_vel_cartimp", 1, &updateLinVel, ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());	
	// self.robot_vel_sub = rospy.Subscriber("/iiwa/ee_vel_cartimp", Pose, self.robot_vel_callback, tcp_nodelay=True)
	ros::Publisher pub_realVel = n.advertise<geometry_msgs::Twist>("/testUnitary/real_velocity", 1000);
	ros::Publisher pub_quat_tracked = n.advertise<geometry_msgs::PoseStamped>("/testUnitary/quat_tracked", 1000);

	ros::Rate loop_rate(1000);

	ros::Duration switch_rate(5);

	ros::Time::now();

	ros::Time time_switch = ros::Time::now() + switch_rate;


	Eigen::Vector3d v_temp = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d A_mat = Eigen::Vector3d(-0.7,-0.7,-0.7);
	Eigen::Matrix3d Actual_A_mat = Eigen::Matrix3d::Identity()*-0.7;
	Eigen::Vector3d w_temp = Eigen::Vector3d(0,0,0);
	while (ros::ok())
	{
		task2_attractor = Eigen::Vector3d(robot_goal[0], robot_goal[1], robot_goal[2]);
		task3_attractor = Eigen::Vector3d(0.8, 0, 0.0); //trash !!!!

		ros::Time time_now = ros::Time::now();

		Task1_velocity.header.stamp = time_now;
		Task1_velocity.header.frame_id = "/world";
		v_temp = getTaskVelocity(robot_state, task1_attractor, A_mat);
		w_temp = getTaskVelocity_Ang(task1_attractor, Actual_A_mat, orientation_des);
		Task1_velocity.twist.linear.x = v_temp(0);
		Task1_velocity.twist.linear.y = v_temp(1);
		Task1_velocity.twist.linear.z = 0;
		Task1_velocity.twist.angular.x = w_temp(0);//world frame
		Task1_velocity.twist.angular.y = w_temp(1);
		Task1_velocity.twist.angular.z = w_temp(2);
		pub_task1.publish(Task1_velocity);

		Task2_velocity.header.stamp = time_now;
		Task2_velocity.header.frame_id = "/world";
		v_temp = getTaskVelocity(robot_state, task2_attractor, A_mat);
		w_temp = getTaskVelocity_Ang(task2_attractor, Actual_A_mat, orientation_des);
		geometry_msgs::Pose quat_tracked;
		quat_tracked.position.x = task2_attractor(0);
		quat_tracked.position.y = task2_attractor(1);
		quat_tracked.position.z = 0.5;
		quat_tracked.orientation.x = orientation_des.x();
		quat_tracked.orientation.y = orientation_des.y();
		quat_tracked.orientation.z = orientation_des.z();
		quat_tracked.orientation.w = orientation_des.w();
		geometry_msgs::PoseStamped quat_tracked_stamped;
		quat_tracked_stamped.pose = quat_tracked;
		quat_tracked_stamped.header.stamp = time_now;
		quat_tracked_stamped.header.frame_id = "/world";
		pub_quat_tracked.publish(quat_tracked_stamped);



		Task2_velocity.twist.linear.x = v_temp(0);
		Task2_velocity.twist.linear.y = v_temp(1);
		Task2_velocity.twist.linear.z = 0;
		Task2_velocity.twist.angular.x = w_temp(0);//world frame
		Task2_velocity.twist.angular.y = w_temp(1);
		Task2_velocity.twist.angular.z = w_temp(2);
		pub_task2.publish(Task2_velocity);

		Task3_velocity.header.stamp = time_now;
		Task3_velocity.header.frame_id = "/world";
		v_temp = getTaskVelocity(robot_state, task3_attractor, A_mat);
		Task3_velocity.twist.linear.x = v_temp(0);
		Task3_velocity.twist.linear.y = v_temp(1);
		Task3_velocity.twist.linear.z = v_temp(2);
		pub_task3.publish(Task3_velocity);

		Task4_velocity.header.stamp = time_now;
		Task4_velocity.header.frame_id = "/world";
		Task4_velocity.twist.linear.x = 0;
		Task4_velocity.twist.linear.y = 0;
		Task4_velocity.twist.linear.z = 0;
		pub_task4.publish(Task4_velocity);

		Real_velocity.linear.x = robot_vel(0);
		Real_velocity.linear.y = robot_vel(1);
		Real_velocity.linear.z = robot_vel(2);
		pub_realVel.publish(Real_velocity);

		ros::spinOnce();

		loop_rate.sleep();

	}




	return 0;
}

