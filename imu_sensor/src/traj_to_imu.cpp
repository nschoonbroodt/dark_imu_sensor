#include "traj_to_imu.hpp"

imu_sensor::msg::RawIMUData traj_to_imu(
    const imu_sensor::msg::Trajectory & last,
    const imu_sensor::msg::Trajectory & previous
) {
    // To avoid warning
    (void) last;
    (void) previous;
    // TODO: not implemented yet
    return imu_sensor::msg::RawIMUData();
}