#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"

/* Used to generate empty trajectory data periodically, just to check that subscription works */
class FakeTrajectoryPublisher : public rclcpp::Node {
public:
    FakeTrajectoryPublisher();
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<imu_sensor::msg::Trajectory>::SharedPtr publisher_;
};