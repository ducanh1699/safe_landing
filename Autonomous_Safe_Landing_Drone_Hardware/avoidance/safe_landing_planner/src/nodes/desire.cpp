#include <mavros_msgs/Trajectory.h>
#include <ros/ros.h>
float x = 0;
float y = 0;
float z = 0;


void fillUnusedTrajectorySetpoints(mavros_msgs::PositionTarget &point) {
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_control");
    ros::NodeHandle n;

    
    ros::Publisher pub = n.advertise<mavros_msgs::Trajectory>("/own/trajectory/desired",10);
  
     
    ros::Rate rate(20);
    
    std::cout<<"Enter target x location ";
    std::cin>>x;
    
    std::cout<<"Enter target y location ";
    std::cin>>y;
    
    std::cout<<"Enter target z location ";
    std::cin>>z;

    mavros_msgs::Trajectory setpoint;

        while(ros::ok())
        {     
              setpoint.header.stamp = ros::Time::now();
              setpoint.header.frame_id = "map";
              setpoint.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
              setpoint.point_1.position.x = x;
              setpoint.point_1.position.y = y;
              setpoint.point_1.position.z = z;
              setpoint.point_1.velocity.x = NAN;
              setpoint.point_1.velocity.y = NAN;
              setpoint.point_1.velocity.z = NAN;
              setpoint.point_1.acceleration_or_force.x = NAN;
              setpoint.point_1.acceleration_or_force.y = NAN;
              setpoint.point_1.acceleration_or_force.z = NAN;
              setpoint.point_1.yaw = NAN;
              setpoint.point_1.yaw_rate = NAN;

              fillUnusedTrajectorySetpoints(setpoint.point_2);
              fillUnusedTrajectorySetpoints(setpoint.point_3);
              fillUnusedTrajectorySetpoints(setpoint.point_4);
              fillUnusedTrajectorySetpoints(setpoint.point_5);

              setpoint.time_horizon = {NAN, NAN, NAN, NAN, NAN};
              setpoint.command = { 17,17, 65535, 65535, 65535};
              bool xy_pos_sp_valid = std::isfinite(setpoint.point_1.position.x) && std::isfinite(setpoint.point_1.position.y);
              bool xy_vel_sp_valid = std::isfinite(setpoint.point_1.velocity.x) && std::isfinite(setpoint.point_1.velocity.y);

              if ((xy_pos_sp_valid || xy_vel_sp_valid) &&
                  (std::isfinite(setpoint.point_1.position.z || std::isfinite(setpoint.point_1.velocity.z)))) {
                setpoint.point_valid = {true, false, false, false, false};
              } else {
                setpoint.point_valid = {false, false, false, false, false};
              }

              pub.publish(setpoint);

            rate.sleep();      
            ros::spinOnce();

            if(!ros::ok())
            break;

        }

    ros::spinOnce();
    return 0;
}
