#include "offboard_control/controller_uav.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_uav");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	velocityCtrl* velocityController = new velocityCtrl(nh, nh_private);

	dynamic_reconfigure::Server<velocity_controller::VelocityControllerConfig> srv;

	dynamic_reconfigure::Server<velocity_controller::VelocityControllerConfig>::CallbackType f;

	f = boost::bind(&velocityCtrl::dynamicReconfigureCallback, velocityController, _1, _2);

	srv.setCallback(f);

	ros::spin();
	return 0;
}
