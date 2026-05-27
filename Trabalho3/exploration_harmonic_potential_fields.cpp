#include "phi_p3dx_mapping/exploration_node.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

/**
 * @brief Exploração autônoma usando Campos Potenciais Harmônicos.
 *
 */
class HarmonicExploration : public ExplorationNode
{
public:
  HarmonicExploration()
  : ExplorationNode("exploration_harmonic_potential_fields_cpp", 0.1)
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

    potential_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/potential_map", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(this->get_logger(),
      "HarmonicExploration iniciado. Aguardando mapa e odometria...");
  }

protected:
  void on_map() override // callback quando há alteração no mapa
  {
    if (!map_msg_) return;

    const auto & info = map_msg_->info;
    const int W   = static_cast<int>(info.width);
    const int H   = static_cast<int>(info.height);
    const int N   = W * H;
    const double res = info.resolution;
    const double ox  = info.origin.position.x;
    const double oy  = info.origin.position.y;

    if (N == 0) return;

    if (static_cast<int>(potential_.size()) != N) {
      potential_.assign(N, 0.5);
    }

    // cell_type: 0=livre  1=obstáculo  2=desconhecido
    std::vector<int8_t> cell_type(N, 2);
    for (int i = 0; i < N; ++i) {
      const int8_t v = map_msg_->data[i];
      if (v < 0) {
        cell_type[i] = 2;
        potential_[i] = 0.0;          // atrativo
      } else if (v >= OCC_THRESH) {
        cell_type[i] = 1;
        potential_[i] = 1.0;          // repulsivo
      } else {
        cell_type[i] = 0;             // livre — será iterado
      }
    }

    for (int iter = 0; iter < N_ITER; ++iter) {
      for (int y = 1; y < H - 1; ++y) {
        for (int x = 1; x < W - 1; ++x) {
          const int idx = x + y * W;
          if (cell_type[idx] != 0) continue;
          potential_[idx] = (potential_[idx + 1] + potential_[idx - 1] +
                             potential_[idx + W] + potential_[idx - W]) / 4.0;
        }
      }
    }

    {
      nav_msgs::msg::OccupancyGrid pot_grid;
      pot_grid.header.stamp    = this->now();
      pot_grid.header.frame_id = map_msg_->header.frame_id;
      pot_grid.info            = info;
      pot_grid.data.resize(N);
      for (int i = 0; i < N; ++i) {
        if (map_msg_->data[i] < 0) {
          pot_grid.data[i] = -1;
        } else {
          const int val = static_cast<int>(std::round(potential_[i] * 100.0));
          pot_grid.data[i] = static_cast<int8_t>(std::clamp(val, 0, 100));
        }
      }
      potential_pub_->publish(pot_grid);
    }

    double robot_wx = 0.0, robot_wy = 0.0;
    bool   tf_ok    = false;

    try {
      geometry_msgs::msg::PoseStamped base_in_base, base_in_map;
      base_in_base.header.frame_id = "base_link";
      base_in_base.header.stamp    = rclcpp::Time(0);
      base_in_base.pose.orientation.w = 1.0;

      tf_buffer_->transform(base_in_base, base_in_map,
                            map_msg_->header.frame_id,
                            tf2::durationFromSec(0.05));

      robot_wx = base_in_map.pose.position.x;
      robot_wy = base_in_map.pose.position.y;

      tf2::Quaternion q;
      tf2::fromMsg(base_in_map.pose.orientation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      robot_yaw_ = yaw;
      tf_ok = true;
    } catch (const tf2::TransformException & ex) {
      if (odom_received_) {
        robot_wx = odom_x_;
        robot_wy = odom_y_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "tf indisponível (%s). Usando odometria.", ex.what());
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "tf e odom indisponíveis. Aguardando...");
        return;
      }
    }

    const int rx = static_cast<int>((robot_wx - ox) / res);
    const int ry = static_cast<int>((robot_wy - oy) / res);

    if (rx < 1 || rx >= W - 1 || ry < 1 || ry >= H - 1) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Robô fora dos limites do mapa (rx=%d, ry=%d).", rx, ry);
      return;
    }

    const int idx_r = rx + ry * W;

    const double gx = (potential_[idx_r + 1] - potential_[idx_r - 1]) / 2.0;
    const double gy = (potential_[idx_r + W] - potential_[idx_r - W]) / 2.0;

    grad_x_       = gx;
    grad_y_       = gy;
    grad_mag_     = std::sqrt(gx * gx + gy * gy);
    robot_world_x_ = robot_wx;
    robot_world_y_ = robot_wy;

    const double dir_angle = std::atan2(-gy, -gx);
    publish_target_pose(robot_wx, robot_wy, dir_angle,
                        map_msg_->header.frame_id);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Robô: (%.2f, %.2f) yaw=%.2f rad | ∇ϕ=(%.3f,%.3f) mag=%.4f dir=%.2f rad",
      robot_wx, robot_wy, robot_yaw_, gx, gy, grad_mag_, dir_angle);

    (void)tf_ok;
  }

  void control_loop() override
  {
    if (laser_ranges_.empty() || !map_msg_) return;
    if (!odom_received_) return;   // aguarda pelo menos uma leitura de odom

    if (grad_mag_ < GRAD_MIN) {
      RCLCPP_INFO_ONCE(this->get_logger(),
        "Exploração concluída: gradiente nulo. Robô parado.");
      stop();
      return;
    }

    const double desired_angle = std::atan2(-grad_y_, -grad_x_);
    double angle_error = desired_angle - robot_yaw_;

    while (angle_error >  M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    const double front_dist = get_front_distance(20.0);

    double w = std::clamp(K_W * angle_error, -W_MAX, W_MAX);
    double v;

    if (front_dist < DANGER_DIST) {
      v = 0.0;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Obstáculo próximo (%.2fm). Girando...", front_dist);
    } else if (front_dist < SAFE_DIST) {
      const double t = (front_dist - DANGER_DIST) / (SAFE_DIST - DANGER_DIST);
      v = std::max(V_MIN, V_MAX * t);
    } else {
      const double alignment = std::cos(angle_error);
      v = std::max(V_MIN, V_MAX * std::max(0.0, alignment));
    }

    publish_velocity(v, w);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_yaw_ = yaw;

    if (!tf_buffer_->canTransform(
          map_msg_ ? map_msg_->header.frame_id : "map",
          "base_link", tf2::TimePointZero))
    {
      robot_yaw_ = odom_yaw_;
    }

    odom_received_ = true;
  }

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double odom_x_   = 0.0;
  double odom_y_   = 0.0;
  double odom_yaw_ = 0.0;
  bool   odom_received_ = false;

  double robot_yaw_     = 0.0;
  double robot_world_x_ = 0.0;
  double robot_world_y_ = 0.0;

  std::vector<double> potential_;
  double grad_x_   = 0.0;
  double grad_y_   = 0.0;
  double grad_mag_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub_;

  static constexpr int    OCC_THRESH  = 50;
  static constexpr int    N_ITER      = 100;
  static constexpr double GRAD_MIN    = 1e-4;
  static constexpr double K_W         = 1.5;
  static constexpr double V_MAX       = 0.25;
  static constexpr double V_MIN       = 0.05;
  static constexpr double W_MAX       = 0.8;
  static constexpr double SAFE_DIST   = 0.6;
  static constexpr double DANGER_DIST = 0.35;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HarmonicExploration>());
  rclcpp::shutdown();
  return 0;
}