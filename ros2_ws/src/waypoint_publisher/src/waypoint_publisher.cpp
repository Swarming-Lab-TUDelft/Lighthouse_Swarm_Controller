#include "waypoint_publisher.hpp"

WaypointPublisher::WaypointPublisher() : Node("waypoint_publisher")
{
  this->waypoint_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>("/waypoints", 10);
  this->pattern_switch_sub_ = this->create_subscription<std_msgs::msg::String>("/ROS_pattern_switch", 10, std::bind(&WaypointPublisher::pattern_switch_callback, this, _1));
  this->timer_ = this->create_wall_timer(1s, std::bind(&WaypointPublisher::timer_callback, this));

  this->current_pattern_function = &generate_rotating_diamond;

}

WaypointPublisher::~WaypointPublisher()
{

}

void WaypointPublisher::timer_callback()
{
  std::vector<Eigen::Vector3d> rotated_vertices = generate_rotating_diamond();

  geometry_msgs::msg::Polygon msg = geometry_msgs::msg::Polygon();
  for (const Eigen::Vector3d& vertex : rotated_vertices)
  {
    geometry_msgs::msg::Point32 point;
    point.x = vertex[0];
    point.y = vertex[1];
    point.z = vertex[2];
    msg.points.push_back(point);
  }

  this->waypoint_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Publishing: ");
  for (const Eigen::Vector3d& vertex : rotated_vertices)
  {
    RCLCPP_INFO(this->get_logger(), "  [%f, %f, %f]", vertex[0], vertex[1], vertex[2]);
  }
}


void WaypointPublisher::pattern_switch_callback(const std_msgs::msg::String::SharedPtr msg) const
{

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointPublisher>());
  rclcpp::shutdown();
  return 0;
}
