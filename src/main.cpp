#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uv_base_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);

	double num_drone;
	nh.getParam("num_drone", num_drone);
	std::unique_ptr<SwarmVehicle> camila(new SwarmVehicle(nh, "camila", num_drone));

	while (ros::ok())
	{
		camila->run();

		ros::spinOnce();
		rate.sleep();
	}

	camila.release();
	return 0;
}