#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"
#include "imu_sensor/msg/imu_data.hpp"

#include "imu_sensor.hpp"

#include "traj_to_imu.hpp"

using namespace std::chrono_literals;

IMUSensor::IMUSensor() : Node("imu_sensor") {
    // Subscribe to trajectory to remember the last 2 points
    this->subscription_ = 
        this->create_subscription<imu_sensor::msg::Trajectory>(
            "topic",
            10,
            [this] (const imu_sensor::msg::Trajectory & trajectory) {
                this->trajectory_callback(trajectory);
            });
    
    // Register a timer (for the moment at constant speed) to publish the IMU Data
    publisher_ = this->create_publisher<imu_sensor::msg::IMUData>("imu_data", 10);
    this->create_timer();

    // declare parameters
    // To react on parameter change
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    
    // Period of message sending
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "The period between two IMU Sensor message, in ms";
    this->declare_parameter("imu_message_period", 10, param_desc);
    this->message_frequency_ = 10 * 1ms;
    // TODO: I have 10ms hardcoded in 2 places! With 2 different units
    auto periodCb = [this](const rclcpp::Parameter &period) {
        std::chrono::milliseconds new_period = period.as_int() * 1ms;
        if (this->message_frequency_ == new_period) {
            return; // no changes
        }
        this->message_frequency_ = new_period;
        this->create_timer();
        // TODO: maybe check allowed timer values
    };
    this->cb_handle_ = param_subscriber_->add_parameter_callback("imu_message_period", periodCb);
};

void IMUSensor::create_timer() {
    timer_ = this->create_wall_timer(message_frequency_, [this]() {
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