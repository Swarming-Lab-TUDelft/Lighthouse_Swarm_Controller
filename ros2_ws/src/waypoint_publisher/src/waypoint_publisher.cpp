#include "waypoint_publisher.hpp"

WaypointPublisher::WaypointPublisher() : Node("waypoint_publisher")
{
  this->waypoint_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>("/waypoints", 10);
  this->pattern_switch_sub_ = this->create_subscription<std_msgs::msg::String>("/ROS_pattern_switch", 10, std::bind(&WaypointPublisher::pattern_switch_callback, this, _1));
  this->timer_ = this->create_wall_timer(1s, std::bind(&WaypointPublisher::timer_callback, this));

  current_pattern_function = &WaypointPublisher::generate_grid;

  this->command_pub_ = this->create_publisher<topic_interface::msg::ControllerCommand>("controller_command", 10);
}

WaypointPublisher::~WaypointPublisher()
{

}

void WaypointPublisher::timer_callback()
{
  std::vector<Eigen::Vector3d> vertices = (this->*current_pattern_function)();

  geometry_msgs::msg::Polygon msg = geometry_msgs::msg::Polygon();
  for (const Eigen::Vector3d& vertex : vertices)
  {
    geometry_msgs::msg::Point32 point;
    point.x = vertex[0];
    point.y = vertex[1];
    point.z = vertex[2];
    msg.points.push_back(point);
  }

  this->waypoint_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Publishing: ");
  for (const Eigen::Vector3d& vertex : vertices)
  {
    RCLCPP_INFO(this->get_logger(), "  [%f, %f, %f]", vertex[0], vertex[1], vertex[2]);
  }
}


void WaypointPublisher::pattern_switch_callback(const std_msgs::msg::String::SharedPtr msg) 
{
  std::string command = msg->data;
  RCLCPP_INFO(this->get_logger(), msg->data.data());
  if (command == std::string("custom/Patterns/activate_pos_commander"))
  {
    this->current_pattern_function = &WaypointPublisher::generate_grid;
    RCLCPP_INFO(this->get_logger(), "generate grid");
  }
  else if (command == std::string("custom/Patterns/activate_vel_commander"))
  {
    //NOT SUPPORTED
    RCLCPP_INFO(this->get_logger(), "NOT SUPPORTED COMMAND");
  }

  else if (command == std::string("custom/Patterns/activate_rotating_diamond"))
  {
    this->current_pattern_function = &WaypointPublisher::generate_rotating_diamond;
    RCLCPP_INFO(this->get_logger(), "generate rotatign diamond");
  }
  else if (command == std::string("custom/Patterns/activate_hor_rotating_lines"))
  {
    this->current_pattern_function = &WaypointPublisher::generate_hor_rotating_lines;
    RCLCPP_INFO(this->get_logger(), "generate horizontal rotating lines");
  }
  else if (command == std::string("custom/Patterns/activate_ver_rotating_lines"))
  {
    this->current_pattern_function = &WaypointPublisher::generate_ver_rotating_lines;
    RCLCPP_INFO(this->get_logger(), "generate vertical rotating lines");
  }  
  else if (command == std::string("custom/Patterns/activate_sin_wave"))
  {
    this->current_pattern_function = &WaypointPublisher::generate_sinwave;
    RCLCPP_INFO(this->get_logger(), "generate sinwave");
  } 
  else if (command == std::string("custom/Patterns/activate_landing_test"))
  {
    this->current_pattern_function = &WaypointPublisher::generate_landing_test;
    RCLCPP_INFO(this->get_logger(), "generate landing test");
  } 
  else
  {
    //NOT IMPLEMENTED
    RCLCPP_INFO(this->get_logger(), "NOT IMPLEMENTED");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointPublisher>());
  rclcpp::shutdown();
  return 0;
}
