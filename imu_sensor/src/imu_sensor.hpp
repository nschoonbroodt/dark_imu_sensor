#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"

class IMUSensor : public rclcpp::Node {
public:
    IMUSensor();
private:
    rclcpp::Subscription<imu_sensor::msg::Trajectory>::SharedPtr subscription_;
    imu_sensor::msg::Trajectory lastTrajectory_;
    imu_sensor::msg::Trajectory secondLastTrajectory_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<imu_sensor::msg::IMUData>::SharedPtr publisher_;

    void trajectory_callback(const imu_sensor::msg::Trajectory & trajectory);
    void imu_message_send();
};


