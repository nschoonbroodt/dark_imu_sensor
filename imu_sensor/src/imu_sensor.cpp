#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"

#include "imu_sensor.hpp"

IMUSensor::IMUSensor() : Node("imu_sensor") {
    auto topic_callback = [this](const imu_sensor::msg::Trajectory & trajectory) {
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << trajectory.timestamp << "'");
    };
    this->subscription_ = this->create_subscription<imu_sensor::msg::Trajectory>("topic", 10, topic_callback);
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUSensor>());
    rclcpp::shutdown();
    return 0;
}