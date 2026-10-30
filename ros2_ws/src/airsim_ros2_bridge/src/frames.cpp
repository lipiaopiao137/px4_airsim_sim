#include "airsim_ros2_bridge/frames.hpp"

#include <cmath>

namespace airsim_bridge
{

namespace
{

const double kNedToEnu[9] = {
  0.0, 1.0, 0.0,
  1.0, 0.0, 0.0,
  0.0, 0.0, -1.0,
};

void multiply(const double a[9], const double b[9], double out[9])
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row * 3 + col] =
        a[row * 3 + 0] * b[0 * 3 + col] +
        a[row * 3 + 1] * b[1 * 3 + col] +
        a[row * 3 + 2] * b[2 * 3 + col];
    }
  }
}

}  // namespace

void quat_to_matrix(double w, double x, double y, double z, double m[9])
{
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (norm == 0.0) {
    m[0] = 1.0; m[1] = 0.0; m[2] = 0.0;
    m[3] = 0.0; m[4] = 1.0; m[5] = 0.0;
    m[6] = 0.0; m[7] = 0.0; m[8] = 1.0;
    return;
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;

  m[0] = 1.0 - 2.0 * (y * y + z * z);
  m[1] = 2.0 * (x * y - z * w);
  m[2] = 2.0 * (x * z + y * w);
  m[3] = 2.0 * (x * y + z * w);
  m[4] = 1.0 - 2.0 * (x * x + z * z);
  m[5] = 2.0 * (y * z - x * w);
  m[6] = 2.0 * (x * z - y * w);
  m[7] = 2.0 * (y * z + x * w);
  m[8] = 1.0 - 2.0 * (x * x + y * y);
}

void matrix_to_quat(const double m[9], double q[4])
{
  const double trace = m[0] + m[4] + m[8];
  double w;
  double x;
  double y;
  double z;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m[7] - m[5]) / s;
    y = (m[2] - m[6]) / s;
    z = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0;
    w = (m[7] - m[5]) / s;
    x = 0.25 * s;
    y = (m[1] + m[3]) / s;
    z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0;
    w = (m[2] - m[6]) / s;
    x = (m[1] + m[3]) / s;
    y = 0.25 * s;
    z = (m[5] + m[7]) / s;
  } else {
    const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0;
    w = (m[3] - m[1]) / s;
    x = (m[2] + m[6]) / s;
    y = (m[5] + m[7]) / s;
    z = 0.25 * s;
  }
  q[0] = w;
  q[1] = x;
  q[2] = y;
  q[3] = z;
}

void ned_pose_to_enu(
  const double p_ned[3], const double q_ned[4],
  double p_enu[3], double q_enu[4])
{
  double rot_ned[9];
  quat_to_matrix(q_ned[0], q_ned[1], q_ned[2], q_ned[3], rot_ned);

  double rot_enu[9];
  multiply(kNedToEnu, rot_ned, rot_enu);

  p_enu[0] = kNedToEnu[0] * p_ned[0] + kNedToEnu[1] * p_ned[1] + kNedToEnu[2] * p_ned[2];
  p_enu[1] = kNedToEnu[3] * p_ned[0] + kNedToEnu[4] * p_ned[1] + kNedToEnu[5] * p_ned[2];
  p_enu[2] = kNedToEnu[6] * p_ned[0] + kNedToEnu[7] * p_ned[1] + kNedToEnu[8] * p_ned[2];

  matrix_to_quat(rot_enu, q_enu);
}

}  // namespace airsim_bridge
