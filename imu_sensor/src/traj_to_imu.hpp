#include "imu_sensor/msg/trajectory.hpp"
#include "imu_sensor/msg/raw_imu_data.hpp"

imu_sensor::msg::RawIMUData traj_to_imu(
    const imu_sensor::msg::Trajectory & last,
    const imu_sensor::msg::Trajectory & previous
);