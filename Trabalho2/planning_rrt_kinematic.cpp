#include "planning_node.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <random>
#include <algorithm>

class PlanningRRTKinematic : public PlanningNode
{
public:
  PlanningRRTKinematic() : PlanningNode("planning_rrt_kinematic_cpp")
  {
    v0_             = 0.4;    // m/s   — velocidade linear constante
    omega_max_      = 0.5;    // rad/s — velocidade angular máxima
    dt_             = 1.0;    // s     — intervalo de simulação
    max_iterations_ = 15000;
    goal_tolerance_ = 0.4;    // m     — raio para considerar goal alcançado
    arc_resolution_ = 0.05;   // m     — resolução para checagem de colisão
    robot_margin_   = 0.35;   // m     — margem de inflação (aumentado para curvas)
    goal_bias_      = 0.1;    // probabilidade de amostrar o goal
    omega_eps_      = 0.01;   // rad/s — limiar para aproximação retilínea
    obstacle_thresh_ = 0.35;  // m
  }

private:
  double v0_, omega_max_, dt_;
  int    max_iterations_;
  double goal_tolerance_;
  double arc_resolution_;
  double robot_margin_;
  double goal_bias_;
  double omega_eps_;
  double obstacle_thresh_;

  struct RRTNode {
    double x, y, theta;
    int    parent_idx;
    double omega;
  };

  std::vector<RRTNode> tree_;
  std::vector<RRTNode> path_;
  bool   rrt_computed_     = false;
  bool   following_path_   = false;
  size_t current_path_idx_ = 0;

  std::mt19937 rng_{std::random_device{}()};

  double normalize_angle(double a) const { return std::atan2(std::sin(a), std::cos(a)); }
  double point_dist(double x1, double y1, double x2, double y2) const { return std::hypot(x2 - x1, y2 - y1); }

  bool is_free(double x, double y) const
  {
    if (!map_msg_) return false;
    const double res      = map_msg_->info.resolution;
    const double origin_x = map_msg_->info.origin.position.x;
    const double origin_y = map_msg_->info.origin.position.y;
    const int    width    = static_cast<int>(map_msg_->info.width);
    const int    height   = static_cast<int>(map_msg_->info.height);
    const int center_col  = static_cast<int>((x - origin_x) / res);
    const int center_row  = static_cast<int>((y - origin_y) / res);
    const int cell_margin = static_cast<int>(std::ceil(robot_margin_ / res));

    for (int dr = -cell_margin; dr <= cell_margin; dr++) {
      for (int dc = -cell_margin; dc <= cell_margin; dc++) {
        if (std::hypot(dc * res, dr * res) > robot_margin_) continue;
        const int col = center_col + dc;
        const int row = center_row + dr;
        if (col < 0 || col >= width || row < 0 || row >= height) return false;
        const int val = map_msg_->data[row * width + col];
        if (val > 50 || val == -1) return false;
      }
    }
    return true;
  }

  void simulate_arc(double x0, double y0, double theta0, double v, double omega, double t,
                    double &xt, double &yt, double &thetat) const
  {
    if (std::abs(omega) < omega_eps_) {
      xt = x0 + v * t * std::cos(theta0);
      yt = y0 + v * t * std::sin(theta0);
      thetat = theta0;
    } else {
      xt = x0 + (v / omega) * (std::sin(theta0 + omega * t) - std::sin(theta0));
      yt = y0 - (v / omega) * (std::cos(theta0 + omega * t) - std::cos(theta0));
      thetat = normalize_angle(theta0 + omega * t);
    }
  }

  bool is_arc_free(double x0, double y0, double theta0, double v, double omega, double delta_t) const
  {
    double xe, ye, thetae;
    simulate_arc(x0, y0, theta0, v, omega, delta_t, xe, ye, thetae);
    const int num_checks = std::max(3, static_cast<int>(point_dist(x0, y0, xe, ye) / arc_resolution_) + 1);
    for (int i = 0; i <= num_checks; i++) {
      double t = static_cast<double>(i) / num_checks * delta_t;
      double xt, yt, thetat;
      simulate_arc(x0, y0, theta0, v, omega, t, xt, yt, thetat);
      if (!is_free(xt, yt)) return false;
    }
    return true;
  }

  int find_nearest(double px, double py) const
  {
    int nearest_idx = 0;
    double min_d = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < tree_.size(); i++) {
      double d = point_dist(tree_[i].x, tree_[i].y, px, py);
      if (d < min_d) { min_d = d; nearest_idx = static_cast<int>(i); }
    }
    return nearest_idx;
  }

  bool compute_rrt()
  {
    if (!map_msg_ || !has_goal()) return false;

    const double res = map_msg_->info.resolution;
    const double origin_x = map_msg_->info.origin.position.x;
    const double origin_y = map_msg_->info.origin.position.y;
    const double map_max_x = origin_x + map_msg_->info.width * res;
    const double map_max_y = origin_y + map_msg_->info.height * res;

    tree_.clear();
    tree_.push_back({x_, y_, theta_, -1, 0.0});

    const double gx = std::get<0>(goal_);
    const double gy = std::get<1>(goal_);

    std::uniform_real_distribution<double> rand_x(origin_x, map_max_x);
    std::uniform_real_distribution<double> rand_y(origin_y, map_max_y);
    std::uniform_real_distribution<double> rand_prob(0.0, 1.0);

    for (int iter = 0; iter < max_iterations_; iter++) {
      double rx, ry;
      if (rand_prob(rng_) < goal_bias_) { rx = gx; ry = gy; }
      else { rx = rand_x(rng_); ry = rand_y(rng_); }

      const int near_idx = find_nearest(rx, ry);
      const RRTNode &qnear = tree_[near_idx];

      const double desired_angle = std::atan2(ry - qnear.y, rx - qnear.x);
      const double angle_diff = normalize_angle(desired_angle - qnear.theta);
      const double omega = std::clamp(angle_diff / dt_, -omega_max_, omega_max_);

      double new_x, new_y, new_theta;
      simulate_arc(qnear.x, qnear.y, qnear.theta, v0_, omega, dt_, new_x, new_y, new_theta);

      if (!is_free(new_x, new_y) || !is_arc_free(qnear.x, qnear.y, qnear.theta, v0_, omega, dt_))
        continue;

      const int new_idx = static_cast<int>(tree_.size());
      tree_.push_back({new_x, new_y, new_theta, near_idx, omega});

      if (point_dist(new_x, new_y, gx, gy) < goal_tolerance_) {
        // Check straight line to exact goal position
        double gdx = gx - new_x, gdy = gy - new_y;
        double gdist = std::hypot(gdx, gdy);
        int nc = std::max(2, static_cast<int>(gdist / arc_resolution_) + 1);
        bool goal_clear = true;
        for (int k = 0; k <= nc; k++) {
          double t = static_cast<double>(k) / nc;
          if (!is_free(new_x + t * gdx, new_y + t * gdy)) { goal_clear = false; break; }
        }

        if (goal_clear) {
          int goal_idx = static_cast<int>(tree_.size());
          tree_.push_back({gx, gy, new_theta, new_idx, 0.0});

          path_.clear();
          int idx = goal_idx;
          while (idx != -1) {
            path_.push_back({tree_[idx].x, tree_[idx].y, tree_[idx].theta,
                             tree_[idx].parent_idx, tree_[idx].omega});
            idx = tree_[idx].parent_idx;
          }
          std::reverse(path_.begin(), path_.end());

          rrt_computed_ = true;
          following_path_ = true;
          current_path_idx_ = 0;

          RCLCPP_INFO(this->get_logger(), "RRT Kinematic found path with %zu waypoints (iter: %d)", path_.size(), iter);
          return true;
        }
      }
    }
    RCLCPP_WARN(this->get_logger(), "RRT Kinematic failed after %d iterations.", max_iterations_);
    rrt_computed_ = false;
    return false;
  }

  void publish_tree()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    nodes_marker_.points.clear();
    edges_marker_.points.clear();

    for (size_t i = 0; i < tree_.size(); i++) {
      geometry_msgs::msg::Point p;
      p.x = tree_[i].x; p.y = tree_[i].y; p.z = 0.0;
      nodes_marker_.points.push_back(p);
    }

    const int ARC_VIS_SEGS = 6;
    for (size_t i = 0; i < tree_.size(); i++) {
      if (tree_[i].parent_idx < 0) continue;
      const RRTNode &parent = tree_[tree_[i].parent_idx];
      for (int s = 0; s < ARC_VIS_SEGS; s++) {
        double t1 = static_cast<double>(s) / ARC_VIS_SEGS * dt_;
        double t2 = static_cast<double>(s + 1) / ARC_VIS_SEGS * dt_;
        double x1, y1, th1, x2, y2, th2;
        simulate_arc(parent.x, parent.y, parent.theta, v0_, tree_[i].omega, t1, x1, y1, th1);
        simulate_arc(parent.x, parent.y, parent.theta, v0_, tree_[i].omega, t2, x2, y2, th2);
        geometry_msgs::msg::Point p1, p2;
        p1.x = x1; p1.y = y1; p1.z = 0.0;
        p2.x = x2; p2.y = y2; p2.z = 0.0;
        edges_marker_.points.push_back(p1);
        edges_marker_.points.push_back(p2);
      }
    }
    marker_array.markers.push_back(nodes_marker_);
    marker_array.markers.push_back(edges_marker_);

    if (!path_.empty()) {
      visualization_msgs::msg::Marker path_edge_marker;
      path_edge_marker.header.frame_id = "map"; path_edge_marker.header.stamp = this->now();
      path_edge_marker.ns = "rrt_path_edges"; path_edge_marker.id = 2;
      path_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      path_edge_marker.action = visualization_msgs::msg::Marker::ADD;
      path_edge_marker.pose.orientation.w = 1.0; path_edge_marker.scale.x = 0.07;
      path_edge_marker.color.r = 0.0; path_edge_marker.color.g = 1.0; path_edge_marker.color.b = 0.0; path_edge_marker.color.a = 1.0;

      for (size_t i = 0; i + 1 < path_.size(); i++) {
        const RRTNode &from = path_[i];
        for (int s = 0; s < ARC_VIS_SEGS; s++) {
          double t1 = static_cast<double>(s) / ARC_VIS_SEGS * dt_;
          double t2 = static_cast<double>(s + 1) / ARC_VIS_SEGS * dt_;
          double x1, y1, th1, x2, y2, th2;
          simulate_arc(from.x, from.y, from.theta, v0_, path_[i+1].omega, t1, x1, y1, th1);
          simulate_arc(from.x, from.y, from.theta, v0_, path_[i+1].omega, t2, x2, y2, th2);
          geometry_msgs::msg::Point p1, p2;
          p1.x = x1; p1.y = y1; p1.z = 0.05;
          p2.x = x2; p2.y = y2; p2.z = 0.05;
          path_edge_marker.points.push_back(p1);
          path_edge_marker.points.push_back(p2);
        }
      }
      marker_array.markers.push_back(path_edge_marker);

      visualization_msgs::msg::Marker path_node_marker;
      path_node_marker.header.frame_id = "map"; path_node_marker.header.stamp = this->now();
      path_node_marker.ns = "rrt_path_nodes"; path_node_marker.id = 3;
      path_node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      path_node_marker.action = visualization_msgs::msg::Marker::ADD;
      path_node_marker.pose.orientation.w = 1.0;
      path_node_marker.scale.x = 0.14; path_node_marker.scale.y = 0.14; path_node_marker.scale.z = 0.14;
      path_node_marker.color.r = 0.0; path_node_marker.color.g = 1.0; path_node_marker.color.b = 0.0; path_node_marker.color.a = 1.0;
      for (size_t i = 0; i < path_.size(); i++) {
        geometry_msgs::msg::Point p; p.x = path_[i].x; p.y = path_[i].y; p.z = 0.05;
        path_node_marker.points.push_back(p);
      }
      marker_array.markers.push_back(path_node_marker);
    }
    marker_pub_->publish(marker_array);
  }

  void on_goal() override
  {
    if (!has_goal()) return;
    if (!map_msg_) { clear_goal(); stop(); return; }
    if (!is_free(std::get<0>(goal_), std::get<1>(goal_))) { clear_goal(); stop(); return; }
    tree_.clear(); path_.clear();
    rrt_computed_ = false; following_path_ = false;
    compute_rrt();
  }

  void control_loop() override
  {
    if (!has_goal()) { stop(); return; }
    if (!rrt_computed_ || !following_path_ || path_.empty()) { stop(); publish_tree(); return; }

    // Checagem de objetivo final alcançado
    const RRTNode &goal_node = path_.back();
    if (point_dist(x_, y_, goal_node.x, goal_node.y) < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Objetivo alcançado!");
      following_path_ = false; clear_goal(); stop(); publish_tree(); return;
    }

    // Segurança reativa
    const double front_dist = get_front_distance(15.0);
    if (front_dist < obstacle_thresh_) {
      stop();
      RCLCPP_WARN(this->get_logger(), "Obstáculo à frente (%.2f m). Parando.", front_dist);
      publish_tree(); return;
    }

    //  1. Encontrar o waypoint mais próximo (somente para frente)
    double min_dist = std::numeric_limits<double>::infinity();
    for (size_t i = current_path_idx_; i < path_.size(); i++) {
      double d = point_dist(x_, y_, path_[i].x, path_[i].y);
      if (d < min_dist) { min_dist = d; current_path_idx_ = i; }
    }

    //  2. Lookahead: encontrar o ponto alvo mais adiante no caminho
    const double LOOKAHEAD = 0.6;
    size_t target_idx = current_path_idx_;
    for (size_t i = current_path_idx_; i < path_.size(); i++) {
      target_idx = i;
      if (point_dist(x_, y_, path_[i].x, path_[i].y) >= LOOKAHEAD) break;
    }

    //  3. Controle reativo cinemático
    double tx = path_[target_idx].x;
    double ty = path_[target_idx].y;

    double dx = tx - x_;
    double dy = ty - y_;
    double angle_to_target = std::atan2(dy, dx);
    double alpha = normalize_angle(angle_to_target - theta_);

    // v = v0 constante, omega proporcional ao erro, limitado pela restrição cinemática
    double omega = std::clamp(alpha * 2.0, -omega_max_, omega_max_);
    publish_velocity(v0_, omega);

    publish_tree();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningRRTKinematic>());
  rclcpp::shutdown();
  return 0;
}
