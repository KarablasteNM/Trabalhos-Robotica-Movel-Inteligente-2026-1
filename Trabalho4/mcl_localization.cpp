#include "phi_p3dx_localization/localization_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <random>
#include <vector>
#include <limits>

/**
 * @class MCLLocalization
 * @brief Implementação do MCL com Ray Casting.
 */
class MCLLocalization : public LocalizationNode
{
public:
  explicit MCLLocalization(const std::string &node_name = "mcl_localization_cpp")
    : LocalizationNode(node_name),
      rng_(std::random_device{}()),
      first_update_(true)
  {
    
    this->declare_parameter<double>("alpha1", 0.10);
    this->declare_parameter<double>("alpha2", 0.01);
    this->declare_parameter<double>("alpha3", 0.01);
    this->declare_parameter<double>("alpha4", 0.10);

    this->declare_parameter<double>("laser_variance", 0.05);
    this->declare_parameter<int>("laser_skip", 10);
    this->declare_parameter<double>("max_laser_range", 5.6);
    this->declare_parameter<double>("laser_offset_x", 0.15);
    this->declare_parameter<double>("laser_offset_y", 0.0);

    alpha1_ = this->get_parameter("alpha1").as_double();
    alpha2_ = this->get_parameter("alpha2").as_double();
    alpha3_ = this->get_parameter("alpha3").as_double();
    alpha4_ = this->get_parameter("alpha4").as_double();
    laser_variance_ = this->get_parameter("laser_variance").as_double();
    laser_skip_ = this->get_parameter("laser_skip").as_int();
    max_laser_range_ = this->get_parameter("max_laser_range").as_double();
    laser_offset_x_ = this->get_parameter("laser_offset_x").as_double();
    laser_offset_y_ = this->get_parameter("laser_offset_y").as_double();

    RCLCPP_INFO(this->get_logger(),
      "[%s] MCL inicializado | α=[%.3f,%.3f,%.3f,%.3f] var=%.4f skip=%d "
      "max_range=%.1f offset=(%.2f,%.2f)",
      node_name.c_str(), alpha1_, alpha2_, alpha3_, alpha4_,
      laser_variance_, laser_skip_, max_laser_range_,
      laser_offset_x_, laser_offset_y_);
  }

  virtual ~MCLLocalization() = default;

private:
  double alpha1_, alpha2_, alpha3_, alpha4_;
  double laser_variance_;
  int    laser_skip_;
  double max_laser_range_;
  double laser_offset_x_, laser_offset_y_;

  std::mt19937 rng_;
  bool first_update_;

  void update_particles() override
  {
    bool has_moved = (std::fabs(u_t.trans) > 1e-5 ||
                      std::fabs(u_t.rot1)  > 1e-5 ||
                      std::fabs(u_t.rot2)  > 1e-5);

    sampling();

    importance_weighting();

    if (has_moved || first_update_) {
      resampling();
      first_update_ = false;
    }
  }

  void sampling()
  {
    const double rot1  = u_t.rot1;
    const double trans = u_t.trans;
    const double rot2  = u_t.rot2;

    if (std::fabs(rot1) < 1e-8 && std::fabs(trans) < 1e-8 && std::fabs(rot2) < 1e-8) {
      return;
    }

    double std_rot1  = std::sqrt(alpha1_ * rot1 * rot1  + alpha2_ * trans * trans);
    double std_trans = std::sqrt(alpha3_ * trans * trans + alpha4_ * (rot1 * rot1 + rot2 * rot2));
    double std_rot2  = std::sqrt(alpha1_ * rot2 * rot2  + alpha2_ * trans * trans);

    std_rot1  = std::max(std_rot1,  1e-6);
    std_trans = std::max(std_trans, 1e-6);
    std_rot2  = std::max(std_rot2,  1e-6);

    std::normal_distribution<double> noise_rot1(0.0, std_rot1);
    std::normal_distribution<double> noise_trans(0.0, std_trans);
    std::normal_distribution<double> noise_rot2(0.0, std_rot2);

    for (auto &p : particles_) {
      const double rot1_hat  = rot1  + noise_rot1(rng_);
      const double trans_hat = trans + noise_trans(rng_);
      const double rot2_hat  = rot2  + noise_rot2(rng_);

      const double theta_prime = p.theta + rot1_hat;
      p.x     += trans_hat * std::cos(theta_prime);
      p.y     += trans_hat * std::sin(theta_prime);
      p.theta  = theta_prime + rot2_hat;

      while (p.theta >  M_PI) p.theta -= 2.0 * M_PI;
      while (p.theta < -M_PI) p.theta += 2.0 * M_PI;
    }
  }

  void importance_weighting()
  {
    if (!z_t || !map_received_ || !current_map_) {
      const double w = 1.0 / static_cast<double>(num_particles_);
      for (auto &p : particles_) p.weight = w;
      return;
    }

    const auto &scan = z_t;
    const auto &map  = current_map_;

    const double resolution = map->info.resolution;
    const double origin_x   = map->info.origin.position.x;
    const double origin_y   = map->info.origin.position.y;
    const int map_width     = static_cast<int>(map->info.width);
    const int map_height    = static_cast<int>(map->info.height);

    const double angle_min       = scan->angle_min;
    const double angle_increment = scan->angle_increment;
    const int    num_readings    = static_cast<int>(scan->ranges.size());

    std::vector<double> beam_offsets;
    beam_offsets.reserve(num_readings / laser_skip_ + 1);
    for (int k = 0; k < num_readings; k += laser_skip_) {
      beam_offsets.push_back(angle_min + k * angle_increment);
    }

    const int FREE_THRESHOLD = 50;

    for (auto &p : particles_) {
      const int col = static_cast<int>(std::floor((p.x - origin_x) / resolution));
      const int row = static_cast<int>(std::floor((p.y - origin_y) / resolution));

      if (col < 0 || col >= map_width || row < 0 || row >= map_height) {
        p.weight = 0.0;
        continue;
      }

      const int idx = row * map_width + col;
      if (idx < 0 || idx >= static_cast<int>(map->data.size())) {
        p.weight = 0.0;
        continue;
      }

      const int8_t cell = map->data[idx];
      if (cell < 0 || cell >= FREE_THRESHOLD) {
        p.weight = 0.0;
        continue;
      }

      const double cos_th = std::cos(p.theta);
      const double sin_th = std::sin(p.theta);
      const double laser_x = p.x + laser_offset_x_ * cos_th - laser_offset_y_ * sin_th;
      const double laser_y = p.y + laser_offset_x_ * sin_th + laser_offset_y_ * cos_th;

      double log_weight = 0.0;

      for (const double &offset : beam_offsets) {
        const int k = static_cast<int>(std::round((offset - angle_min) / angle_increment));
        if (k < 0 || k >= num_readings) continue;

        const double z_k = scan->ranges[k];

        if (!std::isfinite(z_k) || z_k < scan->range_min || z_k > scan->range_max) {
          continue;
        }

        const double beam_angle = p.theta + offset;

        const double z_k_star = ray_casting_dda(laser_x, laser_y, beam_angle,
                                                max_laser_range_, map);

        const double diff = z_k - z_k_star;
        log_weight += -0.5 * diff * diff / laser_variance_;
      }

      p.weight = std::exp(log_weight);
      p.weight = p.weight/std::sqrt(2*M_PI*laser_variance_);
    }

    double weight_sum = 0.0;
    for (const auto &p : particles_) {
      weight_sum += p.weight;
    }

    if (weight_sum > 0.0) {
      for (auto &p : particles_) {
        p.weight /= weight_sum;
      }
    } else {
      const double w = 1.0 / static_cast<double>(num_particles_);
      for (auto &p : particles_) p.weight = w;
    }
  }

  double ray_casting_dda(double ox, double oy, double angle, double max_range,
                         const nav_msgs::msg::OccupancyGrid::SharedPtr &map) const
  {
    const double resolution = map->info.resolution;
    const double origin_x   = map->info.origin.position.x;
    const double origin_y   = map->info.origin.position.y;
    const int width         = static_cast<int>(map->info.width);
    const int height        = static_cast<int>(map->info.height);

    const double dir_x = std::cos(angle);
    const double dir_y = std::sin(angle);

    int current_col = static_cast<int>(std::floor((ox - origin_x) / resolution));
    int current_row = static_cast<int>(std::floor((oy - origin_y) / resolution));

    const int step_col = (dir_x >= 0) ? 1 : -1;
    const int step_row = (dir_y >= 0) ? 1 : -1;

    double t = 0.0;

    double t_max_x, t_max_y;
    double t_delta_x, t_delta_y;

    if (std::fabs(dir_x) < 1e-10) {
      t_max_x   = std::numeric_limits<double>::max();
      t_delta_x = std::numeric_limits<double>::max();
    } else {
      double x_boundary = (step_col > 0) 
          ? (current_col + 1) * resolution + origin_x 
          : current_col * resolution + origin_x;
      t_max_x   = (x_boundary - ox) / dir_x;
      t_delta_x = resolution / std::fabs(dir_x);
    }

    if (std::fabs(dir_y) < 1e-10) {
      t_max_y   = std::numeric_limits<double>::max();
      t_delta_y = std::numeric_limits<double>::max();
    } else {
      double y_boundary = (step_row > 0) 
          ? (current_row + 1) * resolution + origin_y 
          : current_row * resolution + origin_y;
      t_max_y   = (y_boundary - oy) / dir_y;
      t_delta_y = resolution / std::fabs(dir_y);
    }

    if (current_col >= 0 && current_col < width && current_row >= 0 && current_row < height) {
      int idx = current_row * width + current_col;
      int8_t cell = map->data[idx];
      if (cell >= 50 || cell < 0) {
        return 0.0;
      }
    } else {
      return max_range;
    }

    while (t < max_range) {
      if (std::fabs(t_max_x - t_max_y) < 1e-10) {
        t = t_max_x;
        current_col += step_col;
        current_row += step_row;
        t_max_x += t_delta_x;
        t_max_y += t_delta_y;
      } else if (t_max_x < t_max_y) {
        t = t_max_x;
        current_col += step_col;
        t_max_x += t_delta_x;
      } else {
        t = t_max_y;
        current_row += step_row;
        t_max_y += t_delta_y;
      }

      if (t > max_range) {
        return max_range;
      }

      if (current_col < 0 || current_col >= width || current_row < 0 || current_row >= height) {
        return max_range;
      }

      int idx = current_row * width + current_col;
      int8_t cell = map->data[idx];

      if (cell >= 50 || cell < 0) {
        return t;
      }
    }

    return max_range;
  }

  void resampling()
  {
    const int N = static_cast<int>(particles_.size());
    if (N == 0) return;

    std::vector<Particle> new_particles;
    new_particles.reserve(N);

    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0 / N);
    const double r = uniform_dist(rng_);

    double c = particles_[0].weight;
    int i = 0;

    for (int m = 0; m < N; m++) {
      const double U = r + static_cast<double>(m) / N;

      while (U > c && i < N - 1) {
        i++;
        c += particles_[i].weight;
      }

      new_particles.push_back(particles_[i]);
    }

    const double w = 1.0 / static_cast<double>(N);
    for (auto &p : new_particles) {
      p.weight = w;
    }

    particles_ = std::move(new_particles);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MCLLocalization>("mcl_localization");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}