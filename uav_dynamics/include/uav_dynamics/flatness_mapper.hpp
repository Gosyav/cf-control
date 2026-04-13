#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct FlatInput
{
  Eigen::Vector3d pos, vel, acc, jerk, snap;
  double yaw, yaw_rate, yaw_acceleration;
  double mass, gravity;
  double I_xx, I_yy, I_zz;
};

struct FlatOutput
{
  Eigen::Vector3d pos, vel;
  Eigen::Quaterniond quat;
  Eigen::Vector3d omega;
  double thrust;
  Eigen::Vector3d torque;
};

class FlatnessMapper
{
public:
  FlatOutput map(const FlatInput & in) const;
};