#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensor_node");
	ros::NodeHandle n("");
	ros::Rate rate(50);
	ros::Subscriber sub = n.subscribe("/mavros/local_position/pose", 10, &sensor_tfCB);

	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
