#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

using namespace std::chrono_literals;


class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher();
    ~WaypointPublisher();

private:
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    //Geometry functions
    std::vector<Eigen::Vector3d> generate_rotating_diamond();
};