#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"
#include "imu_sensor/msg/imu_data.hpp"
#include "imu_sensor/msg/imu_enums.hpp"

#include "imu_sensor.hpp"

#include "traj_to_imu.hpp"

using namespace std::chrono_literals;

IMUSensor::IMUSensor() : Node("imu_sensor") {

    // Subscribe to trajectory to remember the last 2 points
    this->subscription_ = 
        this->create_subscription<imu_sensor::msg::Trajectory>(
            "trajectory",
            10,
            [this] (const imu_sensor::msg::Trajectory & trajectory) {
                this->trajectory_callback(trajectory);
            });
    
    // Create a publisher to publish data to ros network
    publisher_ = this->create_publisher<imu_sensor::msg::IMUData>("imu_data", 10);

    // declare parameters

    // Param 1: do we send data to ros or to hardware?
    auto ros_or_bus_desc = rcl_interfaces::msg::ParameterDescriptor{};
    ros_or_bus_desc.description = "Sending data through ROS or to com bus";
    this->declare_parameter("message_destination", imu_sensor::msg::IMUEnums::IMU_TO_ROS, ros_or_bus_desc);

    // To react on parameter change
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // On new period parameter subscription, change the timer frequency    
    auto periodCb = [this](const rclcpp::Parameter &period) {
        std::chrono::milliseconds new_period = period.as_int() * 1ms;
        if (this->message_frequency_ == new_period) {
            return; // no changes
        }
        this->message_frequency_ = new_period;
        this->create_task();
        // TODO: maybe check allowed timer values
    };
    this->cb_handle_ = param_subscriber_->add_parameter_callback("imu_message_period", periodCb);

        // declaring the parameter will automatically call the callback (possibly with the configuration value) if done after
    auto imu_period_desc = rcl_interfaces::msg::ParameterDescriptor{};
    imu_period_desc.description = "The period between two IMU Sensor message, in ms";
    this->declare_parameter("imu_message_period", 10, imu_period_desc);

};

void IMUSensor::create_task() {
    RCLCPP_INFO(this->get_logger(), "Create timer with period %d ms", (int)message_frequency_.count());
    timer_ = this->create_timer(message_frequency_, [this]() {
        this->imu_message_send();
    });
}

void IMUSensor::trajectory_callback(const imu_sensor::msg::Trajectory & trajectory) {
    this->secondLastTrajectory_ = this->lastTrajectory_;
    this->lastTrajectory_ = trajectory;
}

void IMUSensor::imu_message_send() {
    auto rawMessage = traj_to_imu(this->lastTrajectory_, this->secondLastTrajectory_);
    auto message = imu_sensor::msg::IMUData();
    message.imu_data = rawMessage;

    this->publisher_->publish(message);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUSensor>());
    rclcpp::shutdown();
    return 0;
}