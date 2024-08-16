#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"

#include "fake_trajectory_publisher.hpp"

using namespace std::chrono_literals;

FakeTrajectoryPublisher::FakeTrajectoryPublisher() : Node("fake_trajectory") {
    publisher_ = this->create_publisher<imu_sensor::msg::Trajectory>("topic", 10);
    auto timer_callback = [this]() {
        auto message = imu_sensor::msg::Trajectory();
        publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(1ms, timer_callback);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
