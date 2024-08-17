#include "rclcpp/rclcpp.hpp"
#include "imu_sensor/msg/trajectory.hpp"
#include "imu_sensor/msg/imu_data.hpp"

#include <chrono>

class IMUSensor : public rclcpp::Node {
public:
    IMUSensor();
private:
    rclcpp::Subscription<imu_sensor::msg::Trajectory>::SharedPtr subscription_;
    imu_sensor::msg::Trajectory lastTrajectory_;
    imu_sensor::msg::Trajectory secondLastTrajectory_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<imu_sensor::msg::IMUData>::SharedPtr publisher_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

    // current (last) frequency of update
    std::chrono::milliseconds message_frequency_;

    void trajectory_callback(const imu_sensor::msg::Trajectory & trajectory);
    void imu_message_send(imu_sensor::msg::IMUData message);
    imu_sensor::msg::IMUData imu_compute_message();
    imu_sensor::msg::IMUData imu_inject_failure(imu_sensor::msg::IMUData initial_message);

    void create_task();

};
