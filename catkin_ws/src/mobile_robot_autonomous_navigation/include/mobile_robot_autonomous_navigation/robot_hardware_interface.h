#ifndef ROBOT_HARDWARE_INTERFACE_H
#define ROBOT_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <std_msgs/Float32.h>
#include <angles/angles.h>

class ROBOTHardwareInterface : public hardware_interface::RobotHW {
public:
    ROBOTHardwareInterface(ros::NodeHandle& nh);
    ~ROBOTHardwareInterface();
    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);

protected:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;


    std::string joint_name_[2] = {"virtual_rear_wheel_joint", "virtual_front_steer_joint"};
    double joint_position_[2];
    double joint_velocity_[2];
    double joint_effort_[2];
    double joint_velocity_command_[2];
    double joint_position_command_[2];

    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // ROS publishers and subscribers
    ros::Publisher throttle_motor_cmd_pub_;
    ros::Publisher steer_motor_cmd_pub_;
    ros::Subscriber throttle_wheel_sub_;
    ros::Subscriber steer_wheel_sub_;

    void throttleWheelCallback(const std_msgs::Float32::ConstPtr& msg);
    void steerWheelCallback(const std_msgs::Float32::ConstPtr& msg);
};

#endif // ROBOT_HARDWARE_INTERFACE_H