#include "planning_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <numeric>

PlanningNode::PlanningNode(const std::string &node_name, double timer_period)
  : Node(node_name),
    x_(0.0), y_(0.0), theta_(0.0),
    laser_angle_min_(0.0), laser_angle_max_(0.0), laser_angle_increment_(0.0),
    goal_(std::make_tuple(0.0, 0.0)), goal_theta_(0.0), has_goal_(false),
    map_msg_(nullptr)
{
  // Publisher: comandos de velocidade
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Subscriber: odometria
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&PlanningNode::odom_callback, this, std::placeholders::_1));

  // Subscriber: scan laser
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "laser_scan", 10,
    std::bind(&PlanningNode::laser_callback, this, std::placeholders::_1));

  // Subscriber: sonar point cloud
  sonar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "sonar_cloud", 10,
    std::bind(&PlanningNode::sonar_callback, this, std::placeholders::_1));

  // Subscriber: objetivo de planejamento
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    std::bind(&PlanningNode::goal_callback, this, std::placeholders::_1));

  // Subscriber: mapa global
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", map_qos,
    std::bind(&PlanningNode::map_callback, this, std::placeholders::_1));

  // Timer para loop de controle
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&PlanningNode::control_loop, this));

  // Publisher para visualizar as arvores de exploracao/planejamento
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tree_markers", 10);

  // Configuracao padrao dos markers de nos (esferas)
  nodes_marker_.header.frame_id = "map";
  nodes_marker_.header.stamp.sec = 0;
  nodes_marker_.header.stamp.nanosec = 0;
  nodes_marker_.ns = "tree_nodes";
  nodes_marker_.id = 0;
  nodes_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes_marker_.action = visualization_msgs::msg::Marker::ADD;
  nodes_marker_.pose.orientation.w = 1.0;
  nodes_marker_.scale.x = 0.1;
  nodes_marker_.scale.y = 0.1;
  nodes_marker_.scale.z = 0.1;
  nodes_marker_.color.r = 1.0;
  nodes_marker_.color.g = 0.0;
  nodes_marker_.color.b = 1.0;
  nodes_marker_.color.a = 1.0;

  // Configuracao padrao dos markers de arestas (linhas)
  edges_marker_.header.frame_id = "map";
  edges_marker_.header.stamp.sec = 0;
  edges_marker_.header.stamp.nanosec = 0;
  edges_marker_.ns = "tree_edges";
  edges_marker_.id = 1;
  edges_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker_.action = visualization_msgs::msg::Marker::ADD;
  edges_marker_.pose.orientation.w = 1.0;
  edges_marker_.scale.x = 0.02; // Espessura da linha
  edges_marker_.color.r = 0.5;
  edges_marker_.color.g = 0.0;
  edges_marker_.color.b = 0.5;
  edges_marker_.color.a = 1.0;

  RCLCPP_INFO(this->get_logger(), "[%s] iniciado — use '2D Nav Goal' no RViz para enviar um goal.", node_name.c_str());
}

void PlanningNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;

  // Extrair yaw do quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, theta_);

  on_odom();
}

void PlanningNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laser_ranges_.clear();
  for (float r : msg->ranges) {
    if (msg->range_min <= r && r <= msg->range_max) {
      laser_ranges_.push_back(r);
    } else {
      laser_ranges_.push_back(std::numeric_limits<float>::infinity());
    }
  }
  laser_angle_min_ = msg->angle_min;
  laser_angle_max_ = msg->angle_max;
  laser_angle_increment_ = msg->angle_increment;

  on_laser();
}

void PlanningNode::sonar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sonar_ranges_.clear();

  int numbytes = msg->data.size();
  int numfields = msg->fields.size();
  int numSonars = msg->width;

  float x,y,z;
    
  sonar_ranges_.clear();
  sonar_angles_.clear();
  for(int n=0; n<numSonars; n++){
      memcpy (&x, &msg->data[n * msg->point_step + msg->fields[0].offset], sizeof (float));
      memcpy (&y, &msg->data[n * msg->point_step + msg->fields[1].offset], sizeof (float));
      memcpy (&z, &msg->data[n * msg->point_step + msg->fields[2].offset], sizeof (float));        
      sonar_ranges_.push_back(sqrt(pow(x,2.0)+pow(y,2.0)));
      sonar_angles_.push_back(atan2(y,x)*180.0/M_PI);
      // RCLCPP_INFO(this->get_logger(), "sonar %d - %d° %.2f",n, sonar_angles_[n], sonar_ranges_[n]);
  }
  

  on_sonar();
}

void PlanningNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_ = std::make_tuple(msg->pose.position.x, msg->pose.position.y);

  // Extrair yaw do quaternion
  tf2::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, goal_theta_);

  has_goal_ = true;

  RCLCPP_INFO(this->get_logger(),
    "Novo goal → x=%.2f y=%.2f θ=%.1f°",
    std::get<0>(goal_), std::get<1>(goal_), goal_theta_ * 180.0 / M_PI);

  on_goal();
}

void PlanningNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_msg_ = msg;
  on_map();
}

std::tuple<double, double, double> PlanningNode::get_pose() const
{
  return std::make_tuple(x_, y_, theta_);
}

bool PlanningNode::has_laser_data() const
{
  return !laser_ranges_.empty();
}

bool PlanningNode::has_sonar_data() const
{
  return !sonar_ranges_.empty();
}

double PlanningNode::get_region_distance(size_t idx_start, size_t idx_end) const
{
  if (!has_laser_data()) {
    return std::numeric_limits<double>::infinity();
  }
  if (idx_start >= laser_ranges_.size() || idx_end >= laser_ranges_.size() || idx_start > idx_end) {
    return std::numeric_limits<double>::infinity();
  }
  auto start_it = laser_ranges_.begin() + idx_start;
  auto end_it = laser_ranges_.begin() + idx_end + 1;
  auto min_it = std::min_element(start_it, end_it);
  return *min_it;
}

double PlanningNode::get_front_distance(double half_angle_deg) const
{
  if (!has_laser_data()) {
    return std::numeric_limits<double>::infinity();
  }
  size_t n = laser_ranges_.size();
  size_t center = n / 2;
  size_t half = static_cast<size_t>(std::abs(half_angle_deg * M_PI / 180.0) / std::max(laser_angle_increment_, 1e-9));
  size_t start = (center > half) ? center - half : 0;
  size_t end = std::min(center + half, n - 1);
  return get_region_distance(start, end);
}

bool PlanningNode::has_goal() const
{
  return has_goal_;
}

void PlanningNode::clear_goal()
{
  goal_ = std::make_tuple(0.0, 0.0);
  has_goal_ = false;
  RCLCPP_INFO(this->get_logger(), "Objetivo concluido.");
}

double PlanningNode::distance_to_goal() const
{
  if (!has_goal()) {
    return std::numeric_limits<double>::infinity();
  }
  double dx = std::get<0>(goal_) - x_;
  double dy = std::get<1>(goal_) - y_;
  return std::hypot(dx, dy);
}

double PlanningNode::angle_to_goal() const
{
  if (!has_goal()) {
    return 0.0;
  }
  double dx = std::get<0>(goal_) - x_;
  double dy = std::get<1>(goal_) - y_;
  double desired = std::atan2(dy, dx);
  double err = desired - theta_;
  return std::atan2(std::sin(err), std::cos(err));
}

void PlanningNode::publish_velocity(double v, double w)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = v;
  twist.angular.z = w;
  cmd_vel_pub_->publish(twist);
}

void PlanningNode::stop()
{
  publish_velocity(0.0, 0.0);
}