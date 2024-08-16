#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"

class IMUSensor : public rclcpp::Node {
public:
    IMUSensor();
private:
    rclcpp::Subscription<imu_sensor::msg::Trajectory>::SharedPtr subscription_;
};


