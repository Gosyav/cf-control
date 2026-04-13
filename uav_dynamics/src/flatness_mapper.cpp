#include "uav_dynamics/flatness_mapper.hpp"

FlatOutput FlatnessMapper::map(const FlatInput & in) const
{
  FlatOutput out;
  out.pos = in.pos;
  out.vel = in.vel;

  Eigen::Vector3d e3(0, 0, 1);

  Eigen::Vector3d a = in.acc + in.gravity * e3;
  Eigen::Vector3d b3 = a.normalized();

  Eigen::Vector3d c1(cos(in.yaw), sin(in.yaw), 0);

  Eigen::Vector3d b2 = b3.cross(c1).normalized();
  Eigen::Vector3d b1 = b2.cross(b3);

  Eigen::Matrix3d R;
  R.col(0) = b1;
  R.col(1) = b2;
  R.col(2) = b3;

  out.quat = Eigen::Quaterniond(R);
  out.quat.normalize();

  out.thrust = in.mass * a.dot(b3);

  // omega
  Eigen::Vector3d jerk_proj = in.jerk - b3 * b3.dot(in.jerk);
  Eigen::Vector3d h = (in.mass / out.thrust) * jerk_proj;

  double p = -h.dot(b2);
  double q = h.dot(b1);
  double r = in.yaw_rate;

  out.omega = Eigen::Vector3d(p, q, r);

  // torque
  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  J(0, 0) = in.I_xx;
  J(1, 1) = in.I_yy;
  J(2, 2) = in.I_zz;

  Eigen::Vector3d omega_dot(0, 0, in.yaw_acceleration);

  out.torque = J * omega_dot + out.omega.cross(J * out.omega);

  return out;
}