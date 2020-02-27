#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

sensor_msgs::Imu imu_msgs;

void sensor_tfCB(const geometry_msgs::PoseStamped &msg){
	static tf::TransformBroadcaster br_ouster, br_depth;
	tf::Transform ouster_transform, depth_transform;
	ouster_transform.setOrigin( tf::Vector3(0, 0, 1) ); //현재 base_link의 좌표를 받아옴.
	depth_transform.setOrigin( tf::Vector3(0, 0, 1) ); //현재 base_link의 좌표를 받아옴.
	
	tf::Quaternion ouster_q, depth_q;
	ouster_q.setRPY( 0, 0, 0);
	depth_q.setRPY( -M_PI_2l, 0, -M_PI_2l);
	ouster_transform.setRotation(ouster_q);
	depth_transform.setRotation(depth_q);
	br_ouster.sendTransform(tf::StampedTransform(ouster_transform, ros::Time::now(), "base_link", "os1_sensor"));
	br_depth.sendTransform(tf::StampedTransform(depth_transform, ros::Time::now(), "base_link", "camera_link"));
}

void odomCB(const nav_msgs::Odometry &msg){

	static tf::TransformBroadcaster br_map;
	tf::Transform map_transform;
	map_transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0) ); //현재 base_link의 좌표를 받아옴.
	
	tf::Quaternion map_q;
	map_q.setRPY( 0, 0, 0);
	map_transform.setRotation(map_q);
	br_map.sendTransform(tf::StampedTransform(map_transform, ros::Time::now(), "map", "odom"));

	static tf::TransformBroadcaster br_odom;
	tf::Transform odom_transform;
	odom_transform.setOrigin( tf::Vector3(0, 0, 0) ); //현재 base_link의 좌표를 받아옴.
	
	tf::Quaternion odom_q;
	odom_q.setRPY( 0, 0, 0);
	odom_transform.setRotation(odom_q);
	br_odom.sendTransform(tf::StampedTransform(odom_transform, ros::Time::now(), "odom", "base_link"));
}

// void imuCB(const sensor_msgs::Imu &msg){
// 	// imu_msgs.header.seq							= msg.header.seq;
// 	imu_msgs.header.frame_id 					= msg.header.frame_id;
// 	imu_msgs.header.stamp.sec					= msg.header.stamp.sec;
// 	imu_msgs.header.stamp.nsec			 		= msg.header.stamp.nsec;
// 	imu_msgs.orientation.x						=  0.001;	//msg.orientation.x;//
// 	imu_msgs.orientation.y						=  -0.01;	//msg.orientation.y;//
// 	imu_msgs.orientation.z						=  0.01;	//msg.orientation.z;//
// 	imu_msgs.orientation.w						=  -0.9999;	//msg.orientation.w;//
// 	imu_msgs.orientation_covariance				= msg.orientation_covariance;
// 	imu_msgs.angular_velocity.x					= 0;//msg.angular_velocity.x;
// 	imu_msgs.angular_velocity.y					= 0;//msg.angular_velocity.y;
// 	imu_msgs.angular_velocity.z					= msg.angular_velocity.z;
// 	imu_msgs.angular_velocity_covariance		= msg.angular_velocity_covariance;
// 	imu_msgs.linear_acceleration.x				= 0;//msg.linear_acceleration.x;
// 	imu_msgs.linear_acceleration.y				= 0;//msg.linear_acceleration.y;
// 	imu_msgs.linear_acceleration.z				= msg.linear_acceleration.z;
// 	imu_msgs.linear_acceleration_covariance		= msg.linear_acceleration_covariance;	
// }


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensor_node");
	ros::NodeHandle n("");
	ros::Rate rate(50);
	// ros::Subscriber odom = n.subscribe("/mavros/local_position/odom", 50, &odomCB);
	// ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_pub",1);
	ros::Subscriber sensor = n.subscribe("/mavros/local_position/pose", 50, &sensor_tfCB);
	// ros::Subscriber imu = n.subscribe("/mavros/imu/data", 50, &imuCB);
	while (ros::ok())
	{
		// imu_pub.publish(imu_msgs);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
