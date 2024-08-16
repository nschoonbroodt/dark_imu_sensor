#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"

class IMUSensor : public rclcpp::Node {
public:
    IMUSensor();
private:
    rclcpp::Subscription<imu_sensor::msg::Trajectory>::SharedPtr subscription_;
    imu_sensor::msg::Trajectory lastTrajectory_;
    imu_sensor::msg::Trajectory secondLastTrajectory_;

    void trajectory_callback(const imu_sensor::msg::Trajectory & trajectory);
};


