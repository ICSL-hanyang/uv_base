#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>




void LocalposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    static tf::TransformBroadcaster br1,br2;
    tf::Transform transform1,transform2;
    transform1.setOrigin( tf::Vector3(0, 0, 0.2) );
    transform2.setOrigin( tf::Vector3(0, 0, 0.2) );
    
    tf::Quaternion q1,q2;
    q1.setRPY(0, 0, 0);
    transform1.setRotation(q1);

    q2.setRPY(0, 0, 0);
    transform2.setRotation(q2);

    // transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "odometry", "depthcamera_link"));
    br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "odometry", "imu_link"));

}


int main(int argc, char** argv){
    ros::init(argc, argv, "depthcamera_link_tf_pub_node");

    ros::NodeHandle node;
    ros::Subscriber local_pose_sub = node.subscribe("/camila1/mavros/local_position/pose", 10, &LocalposeCallback);

    ros::spin();
    return 0;
};
