#include "waypoint_publisher.hpp"

std::vector<Eigen::Vector3d> WaypointPublisher::generate_rotating_diamond()
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

  std::chrono::_V2::system_clock::rep t = std::chrono::steady_clock::now().time_since_epoch().count();
  double angle = 2 * M_PI * (static_cast<double>(t % static_cast<int>(time_interval * 1e9)) / (time_interval * 1e9));

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

  std::vector<Eigen::Vector3d> rotated_vertices;
  for (const Eigen::Vector3d& vertex : vertices)
  {
    rotated_vertices.push_back(rotation_matrix * vertex);
  }

  return rotated_vertices;  
}
