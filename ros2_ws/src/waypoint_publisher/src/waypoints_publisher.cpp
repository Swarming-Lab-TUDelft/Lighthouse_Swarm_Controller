#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

using namespace std::chrono_literals;

class WaypointsPublisher : public rclcpp::Node
{
public:
  WaypointsPublisher()
  : Node("waypoints_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("/waypoints", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&WaypointsPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto rotated_vertices = generate_rotating_diamond();

    auto msg = geometry_msgs::msg::Polygon();
    for (const auto& vertex : rotated_vertices)
    {
      geometry_msgs::msg::Point32 point;
      point.x = vertex[0];
      point.y = vertex[1];
      point.z = vertex[2];
      msg.points.push_back(point);
    }

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: ");
    for (const auto& vertex : rotated_vertices)
    {
      RCLCPP_INFO(this->get_logger(), "  [%f, %f, %f]", vertex[0], vertex[1], vertex[2]);
    }
  }

  std::vector<Eigen::Vector3d> generate_rotating_diamond()
  {
    Eigen::Vector3d center(0.0, 0.0, 1.25);
    double max_distance = 0.75;
    double frequency = 0.1;  // Hz
    double time_interval = 1.0 / frequency;

    Eigen::Vector3d top_vertex = center + Eigen::Vector3d(0, 0, max_distance);
    Eigen::Vector3d bottom_vertex = center - Eigen::Vector3d(0, 0, max_distance);
    Eigen::Vector3d left_vertex = center - Eigen::Vector3d(max_distance, 0, 0);
    Eigen::Vector3d right_vertex = center + Eigen::Vector3d(max_distance, 0, 0);
    Eigen::Vector3d front_vertex = center + Eigen::Vector3d(0, max_distance, 0);
    Eigen::Vector3d back_vertex = center - Eigen::Vector3d(0, max_distance, 0);

    std::vector<Eigen::Vector3d> vertices = {
      top_vertex, bottom_vertex,
      left_vertex, right_vertex,
      front_vertex, back_vertex
    };

    auto t = std::chrono::steady_clock::now().time_since_epoch().count();
    double angle = 2 * M_PI * (static_cast<double>(t % static_cast<int>(time_interval * 1e9)) / (time_interval * 1e9));

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

    std::vector<Eigen::Vector3d> rotated_vertices;
    for (const auto& vertex : vertices)
    {
      rotated_vertices.push_back(rotation_matrix * vertex);
    }

    return rotated_vertices;
  }

  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointsPublisher>());
  rclcpp::shutdown();
  return 0;
}
