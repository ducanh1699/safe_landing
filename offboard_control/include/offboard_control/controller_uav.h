#ifndef CONTROLLER_UAV_H
#define CONTROLLER_UAV_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <ros/subscribe_options.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>


#include <dynamic_reconfigure/server.h>
#include <offboard_control/VelocityControllerConfig.h>

#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

// #include "offboard_control/common.h"
#include "offboard_control/pid_controller_base.h"

using namespace std;
using namespace Eigen;

enum class Frame {
	UAV_NEU_FRAME,
	UAV_BODY_OFFSET_FRAME,
};

struct TransitionPoint {

	double range;
	double atitule;

};

#define NOT_RECEIVED_POSE	0
#define RECEIVED_POSE		1

#define CALCULATED			1
#define NOT_CALCULATED		0

#define ALLOW_DECREASE		1
#define NOT_ALLOW_DECREASE	0

#define ANGLE_1 			15.0
#define ANGLE_2 			8.0
#define ANGLE_3 			4.0


class velocityCtrl
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber multiDOFJointSub_;
        ros::Subscriber mavstateSub_;
        ros::Subscriber mavposeSub_;
        ros::Subscriber mavtwistSub_;
        ros::Subscriber yawreferenceSub_;
        ros::Subscriber target_pose;

		ros::Subscriber marker_pose_sub;
		ros::Subscriber decrese_height_sub;

        ros::ServiceClient arming_client_;
        ros::ServiceClient set_mode_client_;

        ros::Timer markerPosition_timer;
        ros::Timer cmdloop_timer_, statusloop_timer_;
        ros::ServiceServer land_service_;

        Eigen::Vector3d mavPos_, mavVel_, mavRate_;
        Eigen::Vector4d mavAtt_;

		ros::ServiceServer start_land_service_;

        geometry_msgs::Pose home_pose_;

        mavros_msgs::State current_state_;
        mavros_msgs::SetMode offb_set_mode_;
        mavros_msgs::CommandBool arm_cmd_;

        // ros::Publisher trigger_safe_landing;
		ros::Publisher term_PID_z;
		ros::Publisher debug_target;
		ros::Publisher debug_yaw;

        ros::Publisher setRaw_pub;
        ros::Publisher setpoint_pose_pub_; // publish target pose to drone
        ros::Publisher velocity_pub_;
        ros::Time last_request_;

        bool sim_enable_;
        bool received_home_pose;

		double marker_pose_status, calculate_range;
		double range_err;
		struct TransitionPoint sTransitionPoint_1, sTransitionPoint_2, sTransitionPoint_3;

        double error;
        template <class T>
        void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
            ros::Rate pause(hz);
            ROS_INFO_STREAM(msg);
            while (ros::ok() && !(*pred)) {
            ros::spinOnce();
            pause.sleep();
            }
        };

        enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state;
        enum FlightMode {POSITION_MODE, VELOCITY_MODE} flight_mode;

        double max_out_x_, min_out_x_;
		double max_out_y_, min_out_y_;
		double max_out_z_, min_out_z_;
		double max_out_yaw_, min_out_yaw_;
        double kpx_,kdx_, kix_;
        double kpy_,kdy_, kiy_;
        double kpz_,kdz_, kiz_;
        double kpyaw_,kdyaw_, kiyaw_;
		double mavYaw_, curYaw_;

        std::string bodyFrame;
        std::string markerAruco, markerApirlTag, markerWhycon;
        std::string name_;

        double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
        Eigen::Vector3d targetPos_,targetPosPredict_;
		Eigen::Vector4d targetAtt_;
		Eigen::Vector3d markerPosInBodyFrame_;
		Eigen::Matrix3d cam2drone_matrix_;

		bool AllowDecreaseHeight_, StartLanding_;
        // bool trigger_safe_landing_ = false;
        PidControllerBase PID_x, PID_y, PID_z, PID_yaw;

        geometry_msgs::Point whycon_position, aruco_position, apirltag_position;

		std::vector<double> setPoint_[4];

		double period, t;

        boost::shared_ptr<tf::TransformListener> transform_listener;

    public:
        velocityCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        void dynamicReconfigureCallback(velocity_controller::VelocityControllerConfig &config, uint32_t level);
		void yawtargetCallback(const std_msgs::Float64 &msg);
        void mavposeCallback(const geometry_msgs::PoseStamped &msg);
        void statusloopCallback(const ros::TimerEvent &event);
        void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
        void cmdloopCallback(const ros::TimerEvent &event);
        void targetPositioncallback(const geometry_msgs::PoseStamped &msg);
        void pubPosition(const Vector3d &target_position);
        // void pubVelocity(const Eigen::Vector3d &desire_velicity_, double &yaw_rate);
		void pubVelocity(const Eigen::Vector3d &desire_velicity_);
        bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        bool check_position(float error, Eigen::Vector3d current, Eigen::Vector3d target);
		void publish_PIDterm( double pTerm, double iTerm, double dTerm);

		void getErrorDistanceToTarget(const Eigen::Vector3d &target_position, Frame FrameType, Eigen::Vector3d &ErrorDistance);
		// void convertPointFromOffsetBodyToNEU(const Eigen::Vector3d &PointBody, Eigen::Vector3d &PointNEU);

		void ReceivedMarkerPose_Callback(const geometry_msgs::PoseStamped &msg);

		// void CheckAllowDecreaseHeight_Callback(const std_msgs::Bool &msg);

		bool EnableLand_Service(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

		double Query_DecreaseAltitude();

        void markerPositionCallback(const ros::TimerEvent &event);

        void calculate_landing_range();
};

#endif
