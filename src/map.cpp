#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

void odomCB(const geometry_msgs::PoseStamped &msg){
	static tf::TransformBroadcaster br_map;
	tf::Transform map_transform;
	map_transform.setOrigin( tf::Vector3(0, 0, 0) );
	// map_transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0) ); //현재 base_link의 좌표를 받아옴.
	tf::Quaternion map_q;
	map_q.setRPY( 0, 0, 0);
	// map_q.setW(msg.pose.orientation.w);
	// map_q.setX(msg.pose.orientation.x);
	// map_q.setY(msg.pose.orientation.y);
	// map_q.setZ(msg.pose.orientation.z);
	map_transform.setRotation(map_q);
	br_map.sendTransform(tf::StampedTransform(map_transform, ros::Time::now(), "odom", "base_link"));
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_node");
	ros::NodeHandle n("");
	ros::Rate rate(50);

	ros::Subscriber sub_map = n.subscribe("/mavros/local_position/pose", 50, &odomCB);
	while (ros::ok())
	{

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
