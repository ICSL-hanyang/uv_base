#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

float msg_throttle = 0;
float msg_steer = 0;

void joyCB(const sensor_msgs::Joy::ConstPtr &msg)
{
	msg_throttle 	= 	msg->axes.at(1);
	msg_steer		=	msg->axes.at(2);
    ROS_INFO_STREAM("throttle : " << msg_throttle << " steer : " << msg_steer);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_control_node");
	ros::NodeHandle n("");
	ros::Rate rate(10);
    ros::Subscriber joy_sub = n.subscribe("joy", 100, joyCB);

	ros::Publisher joy_pub = n.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 20);

	geometry_msgs::Twist msg_joy_pub;
	while (ros::ok())
	{
		msg_joy_pub.linear.x	=	msg_throttle;
		msg_joy_pub.angular.z	=	msg_steer;
		joy_pub.publish(msg_joy_pub);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
