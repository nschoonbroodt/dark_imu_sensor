#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"

class FakeTrajectoryPublisher : public rclcpp::Node {
public:
    FakeTrajectoryPublisher();
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<imu_sensor::msg::Trajectory>::SharedPtr publisher_;
};