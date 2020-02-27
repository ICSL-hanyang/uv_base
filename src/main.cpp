#include <ros/ros.h>
#include <vehicle.h>
// void setErr(const tf2::Vector3 &err){err_ = err;}
// void targetposeCB(const geometry_msgs::PoseStamped::ConstPtr &msg )
// {
// 	tar_local.header.frame_id	=	msg->header.frame_id;
// 	tar_local.header.seq	=	msg->header.seq;
// 	tar_local.header.stamp	=	msg->header.stamp;
// 	tar_local.pose.orientation.w	=	msg->pose.orientation.w;
// 	tar_local.pose.orientation.x	=	msg->pose.orientation.x;
// 	tar_local.pose.orientation.y	=	msg->pose.orientation.y;
// 	tar_local.pose.orientation.z	=	msg->pose.orientation.z;
// 	tar_local.pose.position.x	=	msg->pose.position.x;
// 	tar_local.pose.position.y	=	msg->pose.position.y;
// 	tar_local.pose.position.z	=	msg->pose.position.z;

// 	target_callback = true;
// 	// ROS_INFO_STREAM("target callback");
// }

void localposeCB(const geometry_msgs::PoseStamped::ConstPtr &msg )
{
	cur_local.header.frame_id	=	msg->header.frame_id;
	cur_local.header.seq	=	msg->header.seq;
	cur_local.header.stamp	=	msg->header.stamp;
	cur_local.pose.orientation.w	=	msg->pose.orientation.w;
	cur_local.pose.orientation.x	=	msg->pose.orientation.x;
	cur_local.pose.orientation.y	=	msg->pose.orientation.y;
	cur_local.pose.orientation.z	=	msg->pose.orientation.z;
	cur_local.pose.position.x	=	msg->pose.position.x;
	cur_local.pose.position.y	=	msg->pose.position.y;
	cur_local.pose.position.z	=	msg->pose.position.z;

	local_callback = true;
	// ROS_INFO_STREAM("local callback");
}

// void velocityCB(const geometry_msgs::TwistStamped::ConstPtr &msg )
// {
// 	cur_linear = msg->twist.linear.x;
// 	cur_angular = msg->twist.angular.z;
// }



// void seek()
// {
// 	tf2::Vector3 err(0, 0, 0);
// 	geometry_msgs::PoseStamped target_pos = tar_local;
// 	geometry_msgs::PoseStamped current_pos = cur_local;
// 	err.setX(target_pos.pose.position.x - current_pos.pose.position.x);
// 	err.setY(target_pos.pose.position.y - current_pos.pose.position.y);
// 	err.setZ(0);

// 	setErr(err);
// }
// double constrain(double val, double min_val, double max_val)
// {
// 	return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
// }

// void gotoVel(double kp_, double ki_, double kd_)
// {
// 	tf2::Vector3 setpoint;
// 	if(target_callback && local_callback)
// 	{
// 		seek();
		
// 		tar_vel.linear.x = constrain( kp_*sqrt( pow(err_.getX(),2.0)+pow(err_.getY(),2.0) ) , 0, 1 );
// 		tar_vel.linear.y = 0;
// 		tar_vel.linear.z = 0;
// 		tar_vel.angular.x = 0;
// 		tar_vel.angular.y = 0;
// 		tar_vel.angular.z = constrain(0.05*err_.getY(),-1,1);
		
// 	}
// 	setpoint_vel_pub.publish(tar_vel);
// 	ROS_INFO_STREAM("cur_linear="<<cur_linear<<std::endl<<"err="<<kp_*err_.getX());
// }
void goalCB(const geometry_msgs::PointStamped::ConstPtr &msg )
{
	if(local_callback){
		setpoint_pose.pose.position.x		=	msg->point.x	+cur_local.pose.position.x;
		setpoint_pose.pose.position.y		=	msg->point.y	+cur_local.pose.position.y;
	}
	ROS_INFO_STREAM("local_x"<<msg->point.x<<"local_y"<<msg->point.y);
	ROS_INFO_STREAM("body_x"<<cur_local.pose.position.x<<"body_y"<<cur_local.pose.position.y);
	ROS_INFO_STREAM("goal_x"<<setpoint_pose.pose.position.x<<"goal_y"<<setpoint_pose.pose.position.y);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uv_base_node");
	ros::NodeHandle n("");
	ros::Rate rate(50);
	// n.getParam("/pid/kp", kp_);
	// n.getParam("/pid/ki", ki_);
	// n.getParam("/pid/kd", kd_);

	// sub_tar_local = n.subscribe("/setpoint_pose",10,&targetposeCB);
	sub_cur_local = n.subscribe("/mavros/local_position/pose",10,&localposeCB);
	// sub_vel_local = n.subscribe("/mavros/local_position/velocity_body",10,&velocityCB);
	sub_goal = n.subscribe("/move_base_simple/goal",10,&goalCB);
	
	// setpoint_vel_pub = n.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped",10);
	setpoint_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
	
	while (ros::ok())
	{
		// gotoVel(kp_, ki_, kd_);
		// if(local_callback)
		// 	setpoint_pose_pub.publish(setpoint_pose);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}