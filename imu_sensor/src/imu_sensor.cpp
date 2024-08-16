#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"

#include "imu_sensor.hpp"

IMUSensor::IMUSensor() : Node("imu_sensor") {
    this->subscription_ = 
        this->create_subscription<imu_sensor::msg::Trajectory>(
            "topic",
            10,
            [this] (const imu_sensor::msg::Trajectory & trajectory) {
                this->trajectory_callback(trajectory);
            });
};

void IMUSensor::trajectory_callback(const imu_sensor::msg::Trajectory & trajectory) {
    this->secondLastTrajectory_ = this->lastTrajectory_;
    this->lastTrajectory_ = trajectory;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUSensor>());
    rclcpp::shutdown();
    return 0;
}