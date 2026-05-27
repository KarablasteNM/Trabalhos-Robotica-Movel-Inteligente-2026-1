#include "phi_p3dx_mapping/mapping_node.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <vector>
#include <algorithm>

/**
 * @brief Mapeamento usando LogOdds.
 *
 */
class MappingLogOdds : public MappingNode
{
public:
  MappingLogOdds() : MappingNode("log_odds_mapping_cpp", 0.1)
  {
    int size =
      map_msg_.info.width *
      map_msg_.info.height;

    log_odds_map_.resize(size, 0.0);

    p_occ_ = 0.75;
    p_free_ = 0.30;

    l_occ_ =
      log(p_occ_ / (1.0 - p_occ_));

    l_free_ =
      log(p_free_ / (1.0 - p_free_));

    l_min_ = -4.0;
    l_max_ = 4.0;

    latest_scan_ =
      std::make_shared<sensor_msgs::msg::LaserScan>();

    scan_sub_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan",
        10,
        std::bind(
          &MappingLogOdds::laser_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "MappingLogOdds initialized");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    scan_sub_;

  sensor_msgs::msg::LaserScan::SharedPtr
    latest_scan_;

  std::vector<double> log_odds_map_;

  double p_occ_;
  double p_free_;

  double l_occ_;
  double l_free_;

  double l_min_;
  double l_max_;

  void laser_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;
  }

  void on_odom() override
  {
    if(latest_scan_->ranges.empty())
      return;

    auto [robot_mx, robot_my] =
      meters_to_cells(x_, y_);

    for(size_t i = 0;
        i < latest_scan_->ranges.size();
        i++)
    {
      double range =
        latest_scan_->ranges[i];

      if(std::isnan(range) ||
         std::isinf(range))
        continue;

      if(range < latest_scan_->range_min ||
         range > latest_scan_->range_max)
        continue;

      double angle =
        theta_ +
        latest_scan_->angle_min +
        i * latest_scan_->angle_increment;

      double hit_x =
        x_ + range * cos(angle);

      double hit_y =
        y_ + range * sin(angle);

      auto [hit_mx, hit_my] =
        meters_to_cells(hit_x, hit_y);

      if(!valid(hit_mx, hit_my))
        continue;

      bresenham(
        robot_mx,
        robot_my,
        hit_mx,
        hit_my);

      update_cell(
        hit_mx,
        hit_my,
        l_occ_);
    }

    update_occupancy_grid();

    publish_map();
  }

  int index(int x, int y) const
  {
    return
      y * map_msg_.info.width + x;
  }

  bool valid(int x, int y) const
  {
    if(x < 0 || y < 0)
      return false;

    if(x >=
       (int)map_msg_.info.width)
      return false;

    if(y >=
       (int)map_msg_.info.height)
      return false;

    return true;
  }

  void update_cell(
    int mx,
    int my,
    double value)
  {
    if(!valid(mx, my))
      return;

    int idx = index(mx, my);

    log_odds_map_[idx] += value;

    log_odds_map_[idx] =
      std::clamp(
        log_odds_map_[idx],
        l_min_,
        l_max_);
  }

  void bresenham(
    int x0,
    int y0,
    int x1,
    int y1)
  {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx =
      x0 < x1 ? 1 : -1;

    int sy =
      y0 < y1 ? 1 : -1;

    int err = dx - dy;

    int x = x0;
    int y = y0;

    while(true)
    {
      if(x == x1 &&
         y == y1)
        break;

      update_cell(
        x,
        y,
        l_free_);

      int e2 = 2 * err;

      if(e2 > -dy)
      {
        err -= dy;
        x += sx;
      }

      if(e2 < dx)
      {
        err += dx;
        y += sy;
      }
    }
  }

  void update_occupancy_grid()
  {
    for(size_t i = 0;
        i < log_odds_map_.size();
        i++)
    {
      double l =
        log_odds_map_[i];

      if(std::abs(l) < 0.01)
      {
        map_msg_.data[i] = -1;
        continue;
      }

      double p =
        1.0 /
        (1.0 + exp(-l));

      int occ =
        (int)(100.0 * p);

      occ =
        std::clamp(
          occ,
          0,
          100);

      map_msg_.data[i] = occ;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MappingLogOdds>());

  rclcpp::shutdown();

  return 0;
}