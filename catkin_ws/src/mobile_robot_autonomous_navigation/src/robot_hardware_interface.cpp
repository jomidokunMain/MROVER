#include <mobile_robot_autonomous_navigation/robot_hardware_interface.h>

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();

    // Initialize ROS publishers and subscribers
    throttle_motor_cmd_pub_ = nh_.advertise<std_msgs::Float32>("throttle_motor_cmd", 1);
    steer_motor_cmd_pub_ = nh_.advertise<std_msgs::Float32>("steer_motor_cmd", 1);
    throttle_wheel_sub_ = nh_.subscribe("throttle_wheel_position", 1, &ROBOTHardwareInterface::throttleWheelCallback, this);
    steer_wheel_sub_ = nh_.subscribe("steer_wheel_position", 1, &ROBOTHardwareInterface::steerWheelCallback, this);

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 10;
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
    // Destructor implementation (even if empty)
}

void ROBOTHardwareInterface::throttleWheelCallback(const std_msgs::Float32::ConstPtr& msg) {
    joint_position_[0] = msg->data;  // Update joint position for throttle wheel
}

void ROBOTHardwareInterface::steerWheelCallback(const std_msgs::Float32::ConstPtr& msg) {
    joint_position_[1] = msg->data;  // Update joint position for steer wheel
}

void ROBOTHardwareInterface::init() {
    for (int i = 0; i < 2; i++) {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create velocity joint interface
        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        position_joint_interface_.registerHandle(jointPositionHandle);
    }

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {
    // Implement as needed, or use feedback callbacks
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
    // Publish motor commands
    std_msgs::Float32 throttle_cmd_msg, steer_cmd_msg;
    throttle_cmd_msg.data = joint_velocity_command_[0];
    steer_cmd_msg.data = joint_velocity_command_[1];

    throttle_motor_cmd_pub_.publish(throttle_cmd_msg);
    steer_motor_cmd_pub_.publish(steer_cmd_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mobile_robot_hardware_interface");
    ros::NodeHandle nh;
    ROBOTHardwareInterface ROBOT(nh);
    ros::MultiThreadedSpinner spinner(2); // Use multiple threads for multiple callbacks
    spinner.spin();
    return 0;
}
