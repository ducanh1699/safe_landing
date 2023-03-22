#include "offboard_control/controller_uav.h"

using namespace Eigen;
using namespace std;

#define ON_MEAS		1
#define ON_ERROR	0

#define PI						3.14159265
#define DELTA_T					0.01
#define USE_MARKER				0
#define USE_BODY_OFFSET			1


Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
	Eigen::Vector3d ev3(p.x, p.y, p.z);
	return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
	Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
	return ev3;
}

int index_setpoint = 0;

velocityCtrl::velocityCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
		: nh_(nh), nh_private_(nh_private), node_state(WAITING_FOR_HOME_POSE), flight_mode(POSITION_MODE), sim_enable_(true)
{
	std::string marker_pose_topic_name;

	transform_listener = boost::make_shared<tf::TransformListener>();

	nh_private_.param("marker_pose_topic", marker_pose_topic_name, std::string("/aruco_detect/pose"));

	mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &velocityCtrl::mavposeCallback, this, ros::TransportHints().tcpNoDelay());

	cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &velocityCtrl::cmdloopCallback,this);  // Define timer for constant loop rate

	markerPosition_timer = nh_.createTimer(ros::Duration(0.01), &velocityCtrl::markerPositionCallback, this);

	setRaw_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

	// trigger_safe_landing = nh_.advertise<std_msgs::Bool>("/trigger_safe_landing", 10);

	target_pose = nh_.subscribe("/target_pos", 10, &velocityCtrl::targetPositioncallback, this, ros::TransportHints().tcpNoDelay());

	setpoint_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	statusloop_timer_ = nh_.createTimer(ros::Duration(0.1), &velocityCtrl::statusloopCallback, this);

	land_service_ = nh_.advertiseService("land", &velocityCtrl::landCallback, this);

	velocity_pub_   = nh_.advertise <geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10 );

	term_PID_z = nh_.advertise<geometry_msgs::Vector3>("/pid_term_z", 10);

	debug_target = nh_.advertise<geometry_msgs::Vector3> ("/debug_term", 10);

	yawreferenceSub_ = nh_.subscribe("reference/yaw", 1, &velocityCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());

	debug_yaw = nh_.advertise<std_msgs::Float64> ("/debug_yaw_term", 10);

	marker_pose_sub = nh_.subscribe
		(marker_pose_topic_name, 1, &velocityCtrl::ReceivedMarkerPose_Callback, this, ros::TransportHints().tcpNoDelay());
	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
	// decrese_height_sub = nh_.subscribe
	// 	("/decrease_height", 1, &velocityCtrl::CheckAllowDecreaseHeight_Callback, this, ros::TransportHints().tcpNoDelay());

	start_land_service_ = nh_.advertiseService("start_landing", &velocityCtrl::EnableLand_Service, this);

	nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
	nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
	nh_private_.param<double>("init_pos_z", initTargetPos_z_, 1.0);

	nh_private_.param<double>("max_out_x", max_out_x_, 0.3);
	nh_private_.param<double>("min_out_x", min_out_x_, -0.3);

	nh_private_.param<double>("Kp_x", kpx_, 1.2);
	nh_private_.param<double>("Ki_x", kix_, 0.3);
	nh_private_.param<double>("Kd_x", kdx_, 0.5);

	nh_private_.param<double>("max_out_y", max_out_y_, 0.3);
	nh_private_.param<double>("min_out_y", min_out_y_, -0.3);

	nh_private_.param<double>("Kp_y", kpy_, 1.6);
	nh_private_.param<double>("Ki_y", kiy_, 0.3);
	nh_private_.param<double>("Kd_y", kdy_, 0.5);

	nh_private_.param<double>("max_out_z", max_out_z_, 0.5);
	nh_private_.param<double>("min_out_z", min_out_z_, -0.5);

	nh_private_.param<double>("Kp_z", kpz_, 1.8);
	nh_private_.param<double>("Ki_z", kiz_, 0.3);
	nh_private_.param<double>("Kd_z", kdz_, 2.0);

	nh_private_.param<double>("max_out", max_out_yaw_, 0.05);
	nh_private_.param<double>("min_out", min_out_yaw_, -0.05);

	nh_private_.param<double>("kp_yaw_", kpyaw_, 0.4);
	nh_private_.param<double>("kd_yaw_", kdyaw_, 0.15);
	nh_private_.param<double>("ki_yaw_", kiyaw_, 0.5);

	nh_private_.param<string>("body_frame", bodyFrame, "/base_link");
	nh_private_.param<string>("aruco_frame", markerAruco, "/aruco_gridboard");
	nh_private_.param<string>("apirl_tag", markerApirlTag, "/apirltag_gridboard");
	nh_private_.param<string>("whycon_frame", markerWhycon, "/whycon");
	/* Get param set points */

	// nh_.getParam("/point1", setPoint_[0]);
	// nh_.getParam("/point2", setPoint_[1]);
	// nh_.getParam("/point3", setPoint_[2]);
	// nh_.getParam("/point4", setPoint_[3]);

	// ROS_INFO_STREAM("Set Point 1: x: " << setPoint_[0].at(0) << " y: " << setPoint_[0].at(1) << " z: " << setPoint_[0].at(2));
	// ROS_INFO_STREAM("Set Point 2: x: " << setPoint_[1].at(0) << " y: " << setPoint_[1].at(1) << " z: " << setPoint_[1].at(2));
	// ROS_INFO_STREAM("Set Point 3: x: " << setPoint_[2].at(0) << " y: " << setPoint_[2].at(1) << " z: " << setPoint_[2].at(2));
	// ROS_INFO_STREAM("Set Point 4: x: " << setPoint_[3].at(0) << " y: " << setPoint_[3].at(1) << " z: " << setPoint_[3].at(2));

	targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;

	PID_x.setKp(kpx_);
	PID_x.setKd(kdx_);
	PID_x.setKi(kix_);
	PID_x.setUMax(max_out_x_);
	PID_x.setUMin(min_out_x_);
	PID_x.SetModeP(ON_ERROR);
	PID_x.SetModeD(ON_ERROR);

	PID_y.setKp(kpy_);
	PID_y.setKd(kdy_);
	PID_y.setKi(kiy_);
	PID_y.setUMax(max_out_y_);
	PID_y.setUMin(min_out_y_);
	PID_y.SetModeP(ON_ERROR);
	PID_y.SetModeD(ON_ERROR);

	PID_z.setKp(kpz_);
	PID_z.setKd(kdz_);
	PID_z.setKi(kiz_);
	PID_z.setUMax(max_out_z_);
	PID_z.setUMin(min_out_z_);
	PID_z.SetModeP(ON_ERROR);
	PID_z.SetModeD(ON_ERROR);

	PID_yaw.setKp(kpyaw_);
	PID_yaw.setKd(kdyaw_);
	PID_yaw.setKi(kiyaw_);
	PID_yaw.setUMax(max_out_yaw_);
	PID_yaw.setUMin(min_out_yaw_);
	PID_yaw.SetModeP(ON_ERROR);
	PID_yaw.SetModeD(ON_ERROR);

	error = 0.15;

	t = 0;
	period = 0;

	cam2drone_matrix_ << 0.0 , -1.0 , 0.0 , -1.0 , 0.0 , 0.0 , 0.0 , 0.0 , -1.0;
	markerPosition_timer.stop();
}

void velocityCtrl::markerPositionCallback(const ros::TimerEvent &event)
{	
	if (mavPos_(2) >= sTransitionPoint_1.atitule)
	{
		if(transform_listener->canTransform(markerWhycon, bodyFrame, ros::Time(0)))
		{
			name_ = markerWhycon;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), whycon_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = whycon_position.x;
			markerPosInBodyFrame_(1) = whycon_position.y;
			markerPosInBodyFrame_(2) = whycon_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Whycon is used in range 1: " << range_err); 

		}
		else if (transform_listener->canTransform(bodyFrame, markerAruco, ros::Time(0)))
		{
			name_ = markerAruco;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), aruco_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = aruco_position.x;
			markerPosInBodyFrame_(1) = aruco_position.y;
			markerPosInBodyFrame_(2) = aruco_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Aruco is used in range 1: " << range_err); 
		}
		else if (transform_listener->canTransform(bodyFrame, markerApirlTag, ros::Time(0)))
		{
			name_ = markerApirlTag;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), apirltag_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = apirltag_position.x;
			markerPosInBodyFrame_(1) = apirltag_position.y;
			markerPosInBodyFrame_(2) = apirltag_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Apirltag is used in range 1: " << range_err); 
		}
		else
		{
			marker_pose_status = NOT_RECEIVED_POSE;
		}
	}
	else if (mavPos_(2) >= sTransitionPoint_2.atitule)
	{
		if (transform_listener->canTransform(bodyFrame, markerAruco, ros::Time(0)))
		{
			name_ = markerAruco;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), aruco_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = aruco_position.x;
			markerPosInBodyFrame_(1) = aruco_position.y;
			markerPosInBodyFrame_(2) = aruco_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Aruco is used in range 2: " << range_err); 
		}
		else if (transform_listener->canTransform(markerWhycon, bodyFrame, ros::Time(0)))
		{
			name_ = markerWhycon;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), whycon_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = whycon_position.x;
			markerPosInBodyFrame_(1) = whycon_position.y;
			markerPosInBodyFrame_(2) = whycon_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Whycon is used in range 2: " << range_err); 
		}
		else if (transform_listener->canTransform(bodyFrame, markerApirlTag, ros::Time(0)))
		{
			name_ = markerApirlTag;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), apirltag_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = apirltag_position.x;
			markerPosInBodyFrame_(1) = apirltag_position.y;
			markerPosInBodyFrame_(2) = apirltag_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Apirltag is used in range 2: " << range_err); 
		}
		else
		{
			marker_pose_status = NOT_RECEIVED_POSE;
		}
	}
	else if (mavPos_(2) >= sTransitionPoint_3.atitule)
	{
		if (transform_listener->canTransform(bodyFrame, markerApirlTag, ros::Time(0)))
		{
			name_ = markerApirlTag;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), apirltag_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = apirltag_position.x;
			markerPosInBodyFrame_(1) = apirltag_position.y;
			markerPosInBodyFrame_(2) = apirltag_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Apirltag is used in range 3: " << range_err); 
		}
		else if (transform_listener->canTransform(bodyFrame, markerAruco, ros::Time(0)))
		{
			name_ = markerAruco;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), aruco_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = aruco_position.x;
			markerPosInBodyFrame_(1) = aruco_position.y;
			markerPosInBodyFrame_(2) = aruco_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Aruco is used in range 3: " << range_err); 
		}
		else if (transform_listener->canTransform(markerWhycon, bodyFrame, ros::Time(0)))
		{
			name_ = markerWhycon;
			tf::StampedTransform transform;
    		transform_listener->lookupTransform(bodyFrame, name_, ros::Time(0), transform);
			tf::pointTFToMsg(transform.getOrigin(), whycon_position);
			marker_pose_status = RECEIVED_POSE;
			markerPosInBodyFrame_(0) = whycon_position.x;
			markerPosInBodyFrame_(1) = whycon_position.y;
			markerPosInBodyFrame_(2) = whycon_position.z;
			range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));
			targetPosPredict_(0) = markerPosInBodyFrame_(0);
			targetPosPredict_(1) = markerPosInBodyFrame_(1);
			ROS_INFO_STREAM ("Whycon is used in range 3: " << range_err); 
		}
		else
		{
			marker_pose_status = NOT_RECEIVED_POSE;
		}
	}
	// if (!transform_listener->canTransform(markerWhycon, bodyFrame, ros::Time(0)))
    //   return;
	// tf::StampedTransform transform;
    // transform_listener->lookupTransform(bodyFrame, markerWhycon,  
    //                            ros::Time(0), transform);
	// tf::pointTFToMsg(transform.getOrigin(), whycon_position);

	// if (!transform_listener->canTransform(bodyFrame, markerAruco, ros::Time(0)))
    //   return;
	// tf::StampedTransform transform1;
    // transform_listener->lookupTransform(bodyFrame, markerAruco,  
    //                            ros::Time(0), transform1);
	// tf::pointTFToMsg(transform1.getOrigin(), aruco_position);

	// if (!transform_listener->canTransform(bodyFrame, markerApirlTag, ros::Time(0)))
    //   return;
	// tf::StampedTransform transform2;
    // transform_listener->lookupTransform(bodyFrame , markerApirlTag,  
    //                            ros::Time(0), transform2);
	// tf::pointTFToMsg(transform2.getOrigin(), apirltag_position);
			
}

void velocityCtrl::ReceivedMarkerPose_Callback(const geometry_msgs::PoseStamped &msg){

	// Eigen::Vector3d markerInCamFrame;
	// /* Marker ----> Drone Frame*/
	// markerInCamFrame << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

	// markerPosInBodyFrame_ = cam2drone_matrix_ * markerInCamFrame;

	// // Round cm
	// markerPosInBodyFrame_(0) = round(markerPosInBodyFrame_(0)*100) / 100;
	// markerPosInBodyFrame_(1) = round(markerPosInBodyFrame_(1)*100) / 100;
	// markerPosInBodyFrame_(2) = round(markerPosInBodyFrame_(2)*100) / 100;

	// targetPosPredict_(0) = markerPosInBodyFrame_(0);
	// targetPosPredict_(1) = markerPosInBodyFrame_(1);

	// range_err = sqrt(pow(markerPosInBodyFrame_(0), 2) + pow(markerPosInBodyFrame_(1), 2));

	// // ROS_INFO_STREAM("Distance to Marker: " << markerPosInBodyFrame_);
	// marker_pose_status = RECEIVED_POSE;
	// std::cout << "point des" << point_des << std::endl;
};

void velocityCtrl::targetPositioncallback(const geometry_msgs::PoseStamped &msg){

	targetPos_ = toEigen(msg.pose.position);
}

void velocityCtrl::yawtargetCallback(const std_msgs::Float64 &msg) {
   mavYaw_ = double(msg.data);
}

// void velocityCtrl::CheckAllowDecreaseHeight_Callback(const std_msgs::Bool &msg){

// 	AllowDecreaseHeight_ = msg.data;
// }

bool velocityCtrl::EnableLand_Service(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {

	ROS_INFO_STREAM("Start landing on marker.");
	StartLanding_ = true;
	markerPosition_timer.start();
	return true;
}

void velocityCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {

	if (!received_home_pose)
	{
		received_home_pose = true;
		home_pose_ = msg.pose;
		ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
	}

	mavPos_ = toEigen(msg.pose.position);
	// Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
	// 						msg.pose.orientation.z);
	// Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0,1,2);
	tf2::Quaternion q;
	double roll, pitch, yaw;

	q.setValue(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	curYaw_ = yaw;
	mavAtt_(0) = msg.pose.orientation.w;
	mavAtt_(1) = msg.pose.orientation.x;
	mavAtt_(2) = msg.pose.orientation.y;
	mavAtt_(3) = msg.pose.orientation.z;
}

void velocityCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {

		mavVel_ = toEigen(msg.twist.linear);
		mavRate_ = toEigen(msg.twist.angular);
}

double velocityCtrl::Query_DecreaseAltitude()
{
	if ((sTransitionPoint_1.range < range_err) && (mavPos_(2) >= sTransitionPoint_1.atitule)) {
		return ALLOW_DECREASE;
	}
	else if ((sTransitionPoint_2.range < range_err) && (mavPos_(2) >= sTransitionPoint_2.atitule)) {
		return ALLOW_DECREASE;
	}
	else if ((sTransitionPoint_3.range < range_err) && (mavPos_(2) >= sTransitionPoint_3.atitule)) {
		return ALLOW_DECREASE;
	}
	else {
		return NOT_ALLOW_DECREASE;
	}
}


void velocityCtrl::statusloopCallback(const ros::TimerEvent &event) {
	// if (sim_enable_) {
	// 	// Enable OFFBoard mode and arm automatically
	// 	// This is only run if the vehicle is simulated
	// 	arm_cmd_.request.value = true;
	// 	offb_set_mode_.request.custom_mode = "OFFBOARD";

	// 	if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {

	// 		if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {

	// 			ROS_INFO("Offboard enabled");
	// 		}

	// 		last_request_ = ros::Time::now();
	// 	}
	// 	else {
	// 		if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {

	// 			if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
	// 				ROS_INFO("Vehicle armed");
	// 			}

	// 			last_request_ = ros::Time::now();
	// 		}
	// 	}
	// }
}

bool velocityCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {

	node_state = LANDING;
	ROS_WARN("Dinh Lam");
	return true;
}

bool velocityCtrl::check_position(float error, Eigen::Vector3d current, Eigen::Vector3d target){

	Eigen::Vector3d stop;
	stop << target - current;
	double a = stop.norm();

	if (a <= error){
		return true;
	}
	else
		return false;
}

void velocityCtrl::calculate_landing_range()
{
	if (StartLanding_ && marker_pose_status == RECEIVED_POSE && calculate_range == NOT_CALCULATED) {

		sTransitionPoint_1.range = mavPos_(2) * tan(ANGLE_1 * PI/180);
		sTransitionPoint_1.atitule = mavPos_(2) * 2/3;

		sTransitionPoint_2.range = mavPos_(2) * tan(ANGLE_2 * PI/180) * 2/3;
		sTransitionPoint_2.atitule = mavPos_(2) * 1/3;

		sTransitionPoint_3.range = mavPos_(2) * tan(ANGLE_3 * PI/180) * 1/3;
		sTransitionPoint_3.atitule = 0.0;

		calculate_range = CALCULATED;

		ROS_INFO("Update MAX MIN PID");
		PID_x.setUMax(0.12);
		PID_x.setUMin(-0.12);

		PID_y.setUMax(0.12);
		PID_y.setUMin(-0.12);

		PID_z.setUMax(0.2);
		PID_z.setUMin(-0.2);
	}
	else
	{
		return;
	}
}

void velocityCtrl::cmdloopCallback(const ros::TimerEvent &event)
{
	switch (node_state) {
		case WAITING_FOR_HOME_POSE:
		{
			waitForPredicate(&received_home_pose, "Waiting for home pose...");
			ROS_INFO("Got pose! Drone Ready to be armed.");
			node_state = MISSION_EXECUTION;
			targetPos_(0) = mavPos_(0);
			targetPos_(1) = mavPos_(1);
			targetPos_(2) = mavPos_(2) + 1.5;
			break;
		}

		case MISSION_EXECUTION:
		{
			switch (flight_mode)
		{
				case POSITION_MODE:
				{
					if(check_position(error, mavPos_, targetPos_))
					{
						flight_mode = VELOCITY_MODE;
					}
					// ROS_INFO("Got pose! Drone Position mode");
					// ROS_INFO_STREAM("Got pose! Drone Velocity x " << targetPos_(0) << " y " << targetPos_(1) << " z " << targetPos_(2));

					pubPosition(targetPos_);

					break;
				}
				case VELOCITY_MODE:
				{
					// ROS_INFO("Got pose! Drone Velocity mode");
					Eigen::Vector3d velocity_vector;
					Eigen::Vector3d ErrorDistance;
					calculate_landing_range();
					if (StartLanding_ && marker_pose_status == RECEIVED_POSE)
					{
						targetPosPredict_(0) = targetPosPredict_(0) - (0.01 * mavVel_(0));
						targetPosPredict_(1) = targetPosPredict_(1) - (0.01 * mavVel_(1));
						if (Query_DecreaseAltitude() == ALLOW_DECREASE)
						{
							targetPos_(2) =  markerPosInBodyFrame_(2);
						}
						else
						{
							targetPos_(2) =  0.0;
						}
						targetPosPredict_(2) = targetPos_(2);
						if (targetPos_(0) != markerPosInBodyFrame_(0) && targetPos_(1) != markerPosInBodyFrame_(1))
						{
							targetPos_(0) = markerPosInBodyFrame_(0);
							targetPos_(1) = markerPosInBodyFrame_(1);
							getErrorDistanceToTarget(targetPos_, Frame::UAV_BODY_OFFSET_FRAME, ErrorDistance);
						}
						else
						{
							// ROS_INFO("Error: [%f, %f]", targetPosPredict_(0), targetPosPredict_(1));
							getErrorDistanceToTarget(targetPosPredict_, Frame::UAV_BODY_OFFSET_FRAME, ErrorDistance);
						}
						if (abs(ErrorDistance(0))< 0.05 && abs(ErrorDistance(1)) < 0.05){
							ErrorDistance(0) = 0.0;
							ErrorDistance(1) = 0.0;
						}
					}
					if(!StartLanding_ || marker_pose_status == NOT_RECEIVED_POSE)
					{
						getErrorDistanceToTarget(targetPos_, Frame::UAV_NEU_FRAME, ErrorDistance);
					}
					/*
					 * UAV_NEU_FRAME: If target is a point in the NEU frame
					 * UAV_BODY_FRAME: If target is a point offset in the BODY frame
					*/

					velocity_vector(0) = PID_x.compute(ErrorDistance(0), 0);
					velocity_vector(1) = PID_y.compute(ErrorDistance(1), 0);
					velocity_vector(2) = PID_z.compute(ErrorDistance(2), 0);

					publish_PIDterm(PID_z.getPTerm(),PID_z.getITerm(), PID_z.getDTerm());

					// debug term
					geometry_msgs::Vector3 msg;
					msg.x = targetPos_(0);
					msg.y = targetPos_(1);
					msg.z = targetPos_(2);
					debug_target.publish(msg);
					// std_msgs::Float64 msg_yaw;
					// msg_yaw.data = mavYaw_;
					// debug_yaw.publish(msg_yaw);
					// pubVelocity(velocity_vector, yaw_rate);
					pubVelocity(velocity_vector);
					if((mavPos_(2) > 15.0) || (mavPos_(2) <= 0.5)){
						node_state = LANDING;
					}
					break;
				}
			}
			break;
		}
		case LANDING:
		{
			offb_set_mode_.request.custom_mode = "AUTO.LAND";
			// trigger_safe_landing_ = true;
			// std_msgs::Bool msg;
			// msg.data = trigger_safe_landing_;
			// trigger_safe_landing.publish(msg);
			if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
			{
				ROS_INFO("[ INFO] --------------- LAND ---------------\n");
			}
			node_state = LANDED;

			ros::spinOnce();

			break;
		}

		case LANDED:
		{
			ROS_INFO("Landed. Please set to position control and disarm.");
			cmdloop_timer_.stop();

			break;
		}
	}
}

void velocityCtrl::pubPosition(const Eigen::Vector3d &target_position){
	geometry_msgs::PoseStamped target_pose_;

	target_pose_.header.stamp = ros::Time::now();

	target_pose_.pose.position.x = target_position(0);
	target_pose_.pose.position.y = target_position(1);
	target_pose_.pose.position.z = target_position(2);

	target_pose_.pose.orientation.w = 1.0;
	target_pose_.pose.orientation.x = 0.0;
	target_pose_.pose.orientation.y = 0.0;
	target_pose_.pose.orientation.z = 0.0;

	setpoint_pose_pub_.publish(target_pose_);
}

void velocityCtrl::publish_PIDterm(double pTerm, double iTerm, double dTerm){
	geometry_msgs::Vector3 msg;
	msg.x = pTerm;
	msg.y = iTerm;
	msg.z = dTerm;
	term_PID_z.publish(msg);
}

void velocityCtrl::getErrorDistanceToTarget(const Eigen::Vector3d &target_position, Frame FrameType, Eigen::Vector3d &ErrorDistance) {

	switch (FrameType) {
		case Frame::UAV_BODY_OFFSET_FRAME: {

			ErrorDistance(0) = target_position(0);
			ErrorDistance(1) = target_position(1);
			ErrorDistance(2) = target_position(2);
		}
			break;
		case Frame::UAV_NEU_FRAME: {

			ErrorDistance(0) = target_position(0) - mavPos_(0);
			ErrorDistance(1) = target_position(1) - mavPos_(1);
			ErrorDistance(2) = target_position(2) - mavPos_(2);
		}
			break;
		default:
			ROS_WARN("Get error distance don't support this Frame");
			break;
	}

}

// void velocityCtrl::convertPointFromOffsetBodyToNEU(const Eigen::Vector3d &PointBody, Eigen::Vector3d &PointNEU) {

// 	Eigen::Vector3d OffsetPointNEU;
// 	OffsetPointNEU = RotationBodyToNEU * PointBody;

// 	PointNEU(0) = OffsetPointNEU(0) + mavPos_(0);
// 	PointNEU(1) = OffsetPointNEU(1) + mavPos_(1);
// 	PointNEU(2) = OffsetPointNEU(2) + mavPos_(2);
// }

// void velocityCtrl::pubVelocity(const Eigen::Vector3d &desire_velicity_, double &yaw_rate_){

// 	mavros_msgs::PositionTarget setpoint_local;

// 	setpoint_local.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

// 	setpoint_local.type_mask = 1991;

// 	setpoint_local.velocity.x = desire_velicity_(0);
// 	setpoint_local.velocity.y = desire_velicity_(1);
// 	setpoint_local.velocity.z = desire_velicity_(2);

// 	setpoint_local.yaw_rate = yaw_rate_;

// 	setRaw_pub.publish(setpoint_local);
// }


void velocityCtrl::pubVelocity(const Eigen::Vector3d &desire_velicity_){

	mavros_msgs::PositionTarget setpoint_local;

	setpoint_local.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

	setpoint_local.type_mask = 1991;

	setpoint_local.velocity.x = desire_velicity_(0);
	setpoint_local.velocity.y = desire_velicity_(1);
	setpoint_local.velocity.z = desire_velicity_(2);

	// setpoint_local.yaw_rate = yaw_rate_;

	setRaw_pub.publish(setpoint_local);
}


void velocityCtrl::dynamicReconfigureCallback(velocity_controller::VelocityControllerConfig &config, uint32_t level) {

	if (PID_x.getKp() != config.kpx) {
		PID_x.setKp(config.kpx);
		ROS_INFO("Reconfigure request : kpx_ = %.2f ", config.kpx);
	}
	else if (PID_x.getKi() != config.kix) {
		PID_x.setKi(config.kix);
		ROS_INFO("Reconfigure request : kix_ = %.2f ", config.kix);
	}
	else if (PID_x.getKd() != config.kdx) {
		PID_x.setKd(config.kdx);
		ROS_INFO("Reconfigure request : kdx_ = %.2f ", config.kdx);
	}

	else if (PID_y.getKp() != config.kpy) {
		PID_y.setKp(config.kpy);
		ROS_INFO("Reconfigure request : kpy_ = %.2f ", config.kpy);
	}
	else if (PID_y.getKi() != config.kiy) {
		PID_y.setKi(config.kiy);
		ROS_INFO("Reconfigure request : kiy_ = %.2f ", config.kiy);
	}
	else if (PID_y.getKd() != config.kdy) {
		PID_y.setKd(config.kdy);
		ROS_INFO("Reconfigure request : kdy_ = %.2f ", config.kdy);
	}



	else if (PID_z.getKp() != config.kpz) {
		PID_z.setKp(config.kpz);
		ROS_INFO("Reconfigure request : kpz_ = %.2f ", config.kpz);
	}
	else if (PID_z.getKi() != config.kiz) {
		PID_z.setKi(config.kiz);
		ROS_INFO("Reconfigure request : kiz_ = %.2f ", config.kiz);
	}
	else if (PID_z.getKd() != config.kdz) {
		PID_z.setKd(config.kdz);
		ROS_INFO("Reconfigure request : kdz_ = %.2f ", config.kdz);
	}



	else if (PID_yaw.getKp() != config.kp_yaw) {
		PID_yaw.setKp(config.kp_yaw);
		ROS_INFO("Reconfigure request : kpyaw_ = %.2f ", config.kp_yaw);
	}
	else if (PID_yaw.getKi() != config.ki_yaw) {
		PID_yaw.setKi(config.ki_yaw);
		ROS_INFO("Reconfigure request : kiyaw_ = %.2f ", config.ki_yaw);
	}
	else if (PID_yaw.getKd() != config.kd_yaw) {
		PID_yaw.setKd(config.kd_yaw);
		ROS_INFO("Reconfigure request : kdyaw_ = %.2f ", config.kd_yaw);
	}

	else if (PID_yaw.getUMax() != config.UMax_yaw) {
		PID_yaw.setUMax(config.UMax_yaw);
		ROS_INFO("Reconfigure request : UMax_yaw = %.2f ", config.UMax_yaw);
	}
	else if (PID_yaw.getUMin() != config.UMin_yaw) {
		PID_yaw.setUMin(config.UMin_yaw);
		ROS_INFO("Reconfigure request : UMin = %.2f ", config.UMin_yaw);
	}

	else if (PID_x.getUMax() != config.UMax_x) {
		PID_x.setUMax(config.UMax_x);
		ROS_INFO("Reconfigure request : UMax_x = %.2f ", config.UMax_x);
	}
	else if (PID_x.getUMin() != config.UMin_x) {
		PID_x.setUMin(config.UMin_x);
		ROS_INFO("Reconfigure request : UMin_x = %.2f ", config.UMin_x);
	}

	else if (PID_y.getUMax() != config.UMax_y) {
		PID_y.setUMax(config.UMax_y);
		ROS_INFO("Reconfigure request : UMax_y = %.2f ", config.UMax_y);
	}
	else if (PID_y.getUMin() != config.UMin_y) {
		PID_y.setUMin(config.UMin_y);
		ROS_INFO("Reconfigure request : UMin_y = %.2f ", config.UMin_y);
	}

	else if (PID_z.getUMax() != config.UMax_z) {
		PID_yaw.setUMax(config.UMax_z);
		ROS_INFO("Reconfigure request : UMax_z = %.2f ", config.UMax_z);
	}

	else if (PID_z.getUMin() != config.UMin_z) {
		PID_z.setUMin(config.UMin_z);
		ROS_INFO("Reconfigure request : UMin_z = %.2f ", config.UMin_z);
	}
}


