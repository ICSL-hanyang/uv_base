#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <uv_base/srvGoToVehicle.h>
#include <uv_base/srvMultiSetpointLocal.h>
#include <uv_base/srvMultiSetpointGlobal.h>

#define CONSTANTS_RADIUS_OF_EARTH 6371000 /* meters (m)		*/
#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)


/* 	Drone modes : 
	MANUAL, STABILIZED,	OFFBOARD, ACRO, ALTCTL,	POSCTL,	RATTITUDE, 
	AUTO.TAKEOFF, AUTO.LAND, AUTO.MISSION, AUTO.LOITER,	AUTO.RTL,
	AUTO.RTGS, AUTO.READY,*/
	
	ros::Subscriber sub_tar_local;
	ros::Subscriber sub_cur_local;
	ros::Subscriber sub_vel_local;
	ros::Subscriber sub_goal;

	/* ros publisher*/
	ros::Publisher setpoint_vel_pub;
	ros::Publisher setpoint_pose_pub;
	
	geometry_msgs::PoseStamped cur_local;
	geometry_msgs::PoseStamped tar_local;
	geometry_msgs::PoseStamped setpoint_pose;
	geometry_msgs::Twist tar_vel;

	tf2::Vector3 err_;
	//velocity
	double cur_angular, cur_linear;

	//param
	double kp_, ki_, kd_;
	bool target_callback = false;
	bool local_callback = false;

	void localposeCB(const geometry_msgs::PoseStamped::ConstPtr &);
	void velocityCB(const geometry_msgs::TwistStamped::ConstPtr &msg );
	void goalCB(const geometry_msgs::PointStamped::ConstPtr &msg );


	/*main drone function*/
	void seek();
	void run(double kp_, double ki_, double kd_);
	double constrain(double val, double min_val, double max_val);


#endif