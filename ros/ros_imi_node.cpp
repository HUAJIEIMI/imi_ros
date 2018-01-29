#include "imi_Driver.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "imi_camera");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");

	imi_wrapper::ImiDriver drv(n, pnh);

	ros::spin();

	return 0;
}
