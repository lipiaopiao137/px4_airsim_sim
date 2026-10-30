// Numeric helpers shared by the AirSim bridge nodes.
//
// Rotation that maps vectors expressed in AirSim's NED world frame into ROS'
// ENU world frame:   E = N_ned,  N = E_ned,  U = -D_ned

#ifndef AIRSIM_ROS2_BRIDGE__FRAMES_HPP_
#define AIRSIM_ROS2_BRIDGE__FRAMES_HPP_

namespace airsim_bridge
{

// Quaternion is (w, x, y, z); the rotation matrix is row-major, m[9].
void quat_to_matrix(double w, double x, double y, double z, double m[9]);

void matrix_to_quat(const double m[9], double q[4]);

// Converts an AirSim NED position and quaternion into the ENU world frame.
void ned_pose_to_enu(
  const double p_ned[3], const double q_ned[4],
  double p_enu[3], double q_enu[4]);

}  // namespace airsim_bridge

#endif  // AIRSIM_ROS2_BRIDGE__FRAMES_HPP_
