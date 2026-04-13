#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uav_dynamics/flatness_mapper.hpp"

using namespace std::chrono_literals;

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode()
  : Node("trajectory_node")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("uav_pose", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&TrajectoryNode::update, this));
  }

private:
  void update()
  {
    double t = this->now().seconds();

    double r = 2.0;
    double w = 0.5;

    double z_amp = 1.0;
    double z_w = 0.5;

    FlatInput in;

    // position
    in.pos = Eigen::Vector3d(r * cos(w * t), r * sin(w * t), 2.0 + z_amp * sin(z_w * t));

    // velocity
    in.vel = Eigen::Vector3d(-r * w * sin(w * t), r * w * cos(w * t), z_amp * z_w * cos(z_w * t));

    // acceleration
    in.acc = Eigen::Vector3d(
      -r * w * w * cos(w * t), -r * w * w * sin(w * t), -z_amp * z_w * z_w * sin(z_w * t));

    in.jerk = Eigen::Vector3d::Zero();
    in.snap = Eigen::Vector3d::Zero();

    in.yaw = 0;
    in.yaw_rate = 0;
    in.yaw_acceleration = 0;

    in.mass = 1.0;
    in.gravity = 9.81;

    in.I_xx = 1.0;
    in.I_yy = 1.0;
    in.I_zz = 1.0;

    auto out = mapper_.map(in);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";

    msg.pose.position.x = out.pos.x();
    msg.pose.position.y = out.pos.y();
    msg.pose.position.z = out.pos.z();

    msg.pose.orientation.x = out.quat.x();
    msg.pose.orientation.y = out.quat.y();
    msg.pose.orientation.z = out.quat.z();
    msg.pose.orientation.w = out.quat.w();

    pub_->publish(msg);
  }

  FlatnessMapper mapper_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryNode>());
  rclcpp::shutdown();
}