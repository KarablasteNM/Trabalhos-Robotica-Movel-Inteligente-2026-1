#include "planning_node.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <random>
#include <algorithm>

class PlanningRRTSimple : public PlanningNode
{
public:
  PlanningRRTSimple() : PlanningNode("planning_rrt_simple_cpp")
  {
    step_size_       = 0.5;    // metros — extensão máxima por passo
    max_iterations_  = 10000;  // número máximo de iterações
    goal_tolerance_  = 0.3;    // metros — raio para considerar goal alcançado
    edge_resolution_ = 0.05;   // metros — resolução para checagem de colisão em arestas
    robot_margin_    = 0.2;    // metros — margem de inflação ao redor do robô
    goal_bias_       = 0.1;    // probabilidade de amostrar o próprio goal

    waypoint_dist_thresh_ = 0.15;   // metros — distância para considerar waypoint atingido
    angle_thresh_         = 0.2;    // rad — ângulo para considerar alinhado
    obstacle_thresh_      = 0.4;    // metros — distância para detectar obstáculo à frente
    linear_speed_         = 0.3;    // m/s
    angular_speed_        = 0.5;    // rad/s
  }

private:
  double step_size_;
  int    max_iterations_;
  double goal_tolerance_;
  double edge_resolution_;
  double robot_margin_;
  double goal_bias_;

  double waypoint_dist_thresh_;
  double angle_thresh_;
  double obstacle_thresh_;
  double linear_speed_;
  double angular_speed_;

  struct RRTNode {
    double x, y;
    int parent_idx;
  };

  std::vector<RRTNode> tree_;
  std::vector<RRTNode> path_;
  bool rrt_computed_  = false;
  bool following_path_ = false;
  size_t current_path_idx_ = 0;

  std::mt19937 rng_{std::random_device{}()};

  // Checagem de colisão

  bool is_free(double x, double y) const
  {
    if (!map_msg_) return false;

    const double res      = map_msg_->info.resolution;
    const double origin_x = map_msg_->info.origin.position.x;
    const double origin_y = map_msg_->info.origin.position.y;
    const int    width    = static_cast<int>(map_msg_->info.width);
    const int    height   = static_cast<int>(map_msg_->info.height);

    const int center_col = static_cast<int>((x - origin_x) / res);
    const int center_row = static_cast<int>((y - origin_y) / res);
    const int cell_margin = static_cast<int>(std::ceil(robot_margin_ / res));

    for (int dr = -cell_margin; dr <= cell_margin; dr++) {
      for (int dc = -cell_margin; dc <= cell_margin; dc++) {
        if (std::hypot(dc * res, dr * res) > robot_margin_) continue;

        const int col = center_col + dc;
        const int row = center_row + dr;

        if (col < 0 || col >= width || row < 0 || row >= height)
          return false;

        const int idx = row * width + col;
        const int val = map_msg_->data[idx];

        if (val > 50 || val == -1)
          return false;
      }
    }
    return true;
  }

  bool is_edge_free(double x1, double y1, double x2, double y2) const
  {
    const double d = std::hypot(x2 - x1, y2 - y1);
    const int num_checks = std::max(2, static_cast<int>(d / edge_resolution_) + 1);

    for (int i = 0; i <= num_checks; i++) {
      const double t  = static_cast<double>(i) / num_checks;
      const double px = x1 + t * (x2 - x1);
      const double py = y1 + t * (y2 - y1);
      if (!is_free(px, py)) return false;
    }
    return true;
  }


  double point_dist(double x1, double y1, double x2, double y2) const
  {
    return std::hypot(x2 - x1, y2 - y1);
  }

  int find_nearest(double px, double py) const
  {
    int    nearest_idx = 0;
    double min_d       = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < tree_.size(); i++) {
      const double d = point_dist(tree_[i].x, tree_[i].y, px, py);
      if (d < min_d) {
        min_d       = d;
        nearest_idx = static_cast<int>(i);
      }
    }
    return nearest_idx;
  }

  //  Suavização de caminho

  void smooth_path()
  {
    if (path_.size() <= 2) return;

    std::vector<RRTNode> smoothed;
    smoothed.push_back(path_.front());

    size_t current = 0;
    while (current < path_.size() - 1) {
      size_t farthest = current + 1;
      for (size_t j = path_.size() - 1; j > current + 1; j--) {
        if (is_edge_free(path_[current].x, path_[current].y,
                         path_[j].x, path_[j].y)) {
          farthest = j;
          break;
        }
      }
      smoothed.push_back(path_[farthest]);
      current = farthest;
    }

    RCLCPP_INFO(this->get_logger(),
      "Caminho suavizado: %zu → %zu waypoints",
      path_.size(), smoothed.size());
    path_ = std::move(smoothed);
  }

  //  Computação da RRT

  /*
   * 1. Amostra pontos aleatórios no mapa (com viés para o goal).
   * 2. Encontra o nodo mais próximo e expande em sua direção (step_size_).
   * 3. Verifica colisão do novo nodo e da aresta.
   * 4. Se o novo nodo estiver dentro de goal_tolerance_ do goal, conecta ao goal.
   * 5. Faz backtracking para obter o caminho e suaviza.
   */
  bool compute_rrt()
  {
    if (!map_msg_ || !has_goal()) return false;

    const double res      = map_msg_->info.resolution;
    const double origin_x = map_msg_->info.origin.position.x;
    const double origin_y = map_msg_->info.origin.position.y;
    const int    width    = static_cast<int>(map_msg_->info.width);
    const int    height   = static_cast<int>(map_msg_->info.height);

    const double map_max_x = origin_x + width  * res;
    const double map_max_y = origin_y + height * res;

    tree_.clear();
    tree_.push_back({x_, y_, -1});

    const double gx = std::get<0>(goal_);
    const double gy = std::get<1>(goal_);

    std::uniform_real_distribution<double> rand_x(origin_x, map_max_x);
    std::uniform_real_distribution<double> rand_y(origin_y, map_max_y);
    std::uniform_real_distribution<double> rand_prob(0.0, 1.0);

    RCLCPP_INFO(this->get_logger(),
      "Iniciando RRT: inicio=(%.2f,%.2f), goal=(%.2f,%.2f), step=%.2f, max_iter=%d",
      x_, y_, gx, gy, step_size_, max_iterations_);

    for (int iter = 0; iter < max_iterations_; iter++) {
      // 1. Amostrar ponto aleatório (com goal bias)
      double rx, ry;
      if (rand_prob(rng_) < goal_bias_) {
        rx = gx;
        ry = gy;
      } else {
        rx = rand_x(rng_);
        ry = rand_y(rng_);
      }

      // 2. Encontrar nodo mais próximo
      const int near_idx = find_nearest(rx, ry);
      const double near_x = tree_[near_idx].x;
      const double near_y = tree_[near_idx].y;

      // 3. Estender em direção ao ponto amostrado (limitado por step_size_)
      const double d = point_dist(near_x, near_y, rx, ry);
      double new_x, new_y;

      if (d <= step_size_) {
        new_x = rx;
        new_y = ry;
      } else {
        new_x = near_x + step_size_ * (rx - near_x) / d;
        new_y = near_y + step_size_ * (ry - near_y) / d;
      }

      // 4. Verificação de colisão: nodo e aresta
      if (!is_free(new_x, new_y)) continue;
      if (!is_edge_free(near_x, near_y, new_x, new_y)) continue;

      // 5. Inserir novo nodo na árvore
      const int new_node_idx = static_cast<int>(tree_.size());
      tree_.push_back({new_x, new_y, near_idx});

      // 6. Checagem de aproximação do goal
      if (point_dist(new_x, new_y, gx, gy) < goal_tolerance_) {
        // Tentar conectar diretamente ao goal
        if (is_edge_free(new_x, new_y, gx, gy)) {
          tree_.push_back({gx, gy, new_node_idx});

          // 7. Backtracking: reconstruir caminho do goal até a raiz
          path_.clear();
          int idx = static_cast<int>(tree_.size()) - 1;
          while (idx != -1) {
            path_.push_back({tree_[idx].x, tree_[idx].y, idx});
            idx = tree_[idx].parent_idx;
          }
          std::reverse(path_.begin(), path_.end());

          // Suavizar o caminho
          smooth_path();

          rrt_computed_    = true;
          following_path_  = true;
          current_path_idx_ = 1;  // índice 0 é a posição inicial

          RCLCPP_INFO(this->get_logger(),
            "RRT encontrou caminho com %zu waypoints (árvore: %zu nodos, iteração: %d)",
            path_.size(), tree_.size(), iter);
          return true;
        }
      }
    }

    RCLCPP_WARN(this->get_logger(),
      "RRT falhou após %d iterações (árvore: %zu nodos).",
      max_iterations_, tree_.size());
    rrt_computed_ = false;
    return false;
  }

  //  Visualização
  void publish_tree()
  {
    visualization_msgs::msg::MarkerArray marker_array;

    nodes_marker_.points.clear();
    edges_marker_.points.clear();

    for (size_t i = 0; i < tree_.size(); i++) {
      geometry_msgs::msg::Point p;
      p.x = tree_[i].x;
      p.y = tree_[i].y;
      p.z = 0.0;
      nodes_marker_.points.push_back(p);

      if (tree_[i].parent_idx >= 0) {
        geometry_msgs::msg::Point pp;
        pp.x = tree_[tree_[i].parent_idx].x;
        pp.y = tree_[tree_[i].parent_idx].y;
        pp.z = 0.0;
        edges_marker_.points.push_back(pp);
        edges_marker_.points.push_back(p);
      }
    }

    marker_array.markers.push_back(nodes_marker_);
    marker_array.markers.push_back(edges_marker_);

    if (!path_.empty()) {
      visualization_msgs::msg::Marker path_edge_marker;
      path_edge_marker.header.frame_id = "map";
      path_edge_marker.header.stamp    = this->now();
      path_edge_marker.ns  = "rrt_path_edges";
      path_edge_marker.id  = 2;
      path_edge_marker.type   = visualization_msgs::msg::Marker::LINE_LIST;
      path_edge_marker.action = visualization_msgs::msg::Marker::ADD;
      path_edge_marker.pose.orientation.w = 1.0;
      path_edge_marker.scale.x = 0.06;  // espessura
      path_edge_marker.color.r = 0.0;
      path_edge_marker.color.g = 1.0;
      path_edge_marker.color.b = 0.0;
      path_edge_marker.color.a = 1.0;

      for (size_t i = 0; i + 1 < path_.size(); i++) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = path_[i].x;     p1.y = path_[i].y;     p1.z = 0.05;
        p2.x = path_[i+1].x;   p2.y = path_[i+1].y;   p2.z = 0.05;
        path_edge_marker.points.push_back(p1);
        path_edge_marker.points.push_back(p2);
      }
      marker_array.markers.push_back(path_edge_marker);

      visualization_msgs::msg::Marker path_node_marker;
      path_node_marker.header.frame_id = "map";
      path_node_marker.header.stamp    = this->now();
      path_node_marker.ns  = "rrt_path_nodes";
      path_node_marker.id  = 3;
      path_node_marker.type   = visualization_msgs::msg::Marker::SPHERE_LIST;
      path_node_marker.action = visualization_msgs::msg::Marker::ADD;
      path_node_marker.pose.orientation.w = 1.0;
      path_node_marker.scale.x = 0.12;
      path_node_marker.scale.y = 0.12;
      path_node_marker.scale.z = 0.12;
      path_node_marker.color.r = 0.0;
      path_node_marker.color.g = 1.0;
      path_node_marker.color.b = 0.0;
      path_node_marker.color.a = 1.0;

      for (size_t i = 0; i < path_.size(); i++) {
        geometry_msgs::msg::Point p;
        p.x = path_[i].x;
        p.y = path_[i].y;
        p.z = 0.05;
        path_node_marker.points.push_back(p);
      }
      marker_array.markers.push_back(path_node_marker);
    }

    marker_pub_->publish(marker_array);
  }

  //  Callbacks

  void on_goal() override
  {
    if (!has_goal()) return;

    if (!map_msg_) {
      RCLCPP_WARN(this->get_logger(), "Goal recebido mas sem mapa. Ignorando.");
      clear_goal();
      stop();
      return;
    }

    const double gx = std::get<0>(goal_);
    const double gy = std::get<1>(goal_);

    if (!is_free(gx, gy)) {
      RCLCPP_WARN(this->get_logger(),
        "Goal (%.2f, %.2f) está em espaço ocupado. Ignorando.", gx, gy);
      clear_goal();
      stop();
      return;
    }

    tree_.clear();
    path_.clear();
    rrt_computed_   = false;
    following_path_ = false;

    RCLCPP_INFO(this->get_logger(),
      "Novo goal recebido (%.2f, %.2f). Computando RRT...", gx, gy);

    compute_rrt();
  }

  void control_loop() override
  {
    if (!has_goal()) {
      stop();
      return;
    }

    if (!rrt_computed_ || !following_path_) {
      stop();
      publish_tree();
      return;
    }

    if (current_path_idx_ >= path_.size()) {
      RCLCPP_INFO(this->get_logger(), "Objetivo alcançado!");
      following_path_ = false;
      clear_goal();
      stop();
      publish_tree();
      return;
    }

    const double target_x = path_[current_path_idx_].x;
    const double target_y = path_[current_path_idx_].y;

    const double dx = target_x - x_;
    const double dy = target_y - y_;
    const double dist_to_wp = std::hypot(dx, dy);

    const double desired   = std::atan2(dy, dx);
    const double angle_err = std::atan2(std::sin(desired - theta_),
                                         std::cos(desired - theta_));

    if (dist_to_wp < waypoint_dist_thresh_) {
      current_path_idx_++;
      if (current_path_idx_ >= path_.size()) {
        RCLCPP_INFO(this->get_logger(), "Objetivo alcançado!");
        following_path_ = false;
        clear_goal();
        stop();
      }
      publish_tree();
      return;
    }

    const double front_dist = get_front_distance(15.0);
    if (front_dist < obstacle_thresh_) {
      stop();
      RCLCPP_WARN(this->get_logger(), "Obstáculo à frente (%.2f m). Parando.", front_dist);
      publish_tree();
      return;
    }

    if (std::abs(angle_err) > angle_thresh_) {
      const double w = std::clamp(angle_err * 2.0, -angular_speed_, angular_speed_);
      publish_velocity(0.0, w);
    } else {
      publish_velocity(linear_speed_, 0.0);
    }

    publish_tree();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningRRTSimple>());
  rclcpp::shutdown();
  return 0;
}
