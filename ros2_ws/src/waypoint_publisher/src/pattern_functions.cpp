#include "waypoint_publisher.hpp"

using namespace Eigen; 

std::vector<Vector3d> WaypointPublisher::generate_grid()
{
    int no_drones = 8;
    double spacing = 0.5;
    double height = 1.0;
    Eigen::Vector2d offset = Eigen::Vector2d(0.0, 0.0);

    int grid_size = std::ceil(std::sqrt(no_drones));
    std::vector<Vector3d> grid;
    for (int x = 0; x < grid_size; ++x)
    {
        for (int y = 0; y < grid_size; ++y)
        {
            grid.push_back(Vector3d(
                (x - (grid_size - 1) / 2.0) * spacing + offset[0],
                (y - (grid_size - 1) / 2.0) * spacing + offset[1],
                height));
        }
    }
    return grid;
}

Vector3d WaypointPublisher::generate_velocities(Vector3d pos, Vector3d vel, double height, double turn_scaler, double set_speed)
{
    Vector3d v_origin = Vector3d(0.0, 0.0, height) - pos;
    Vector3d a = v_origin - (v_origin.dot(vel) / vel.squaredNorm()) * vel;
    a = a.normalized() * turn_scaler;
    Vector3d new_vel = vel + a;

    if (set_speed > 0)
    {
        new_vel = new_vel.normalized() * set_speed;
    }

    return new_vel;
}

std::vector<Vector3d> WaypointPublisher::generate_rotating_diamond()
{
Vector3d center(0.0, 0.0, 1.25);
  double max_distance = 0.75;
  double frequency = 0.1;  // Hz
  double time_interval = 1.0 / frequency;

  Vector3d top_vertex = center +  Vector3d(0, 0, max_distance);
  Vector3d bottom_vertex = center -  Vector3d(0, 0, max_distance);
  Vector3d left_vertex = center -  Vector3d(max_distance, 0, 0);
  Vector3d right_vertex = center +  Vector3d(max_distance, 0, 0);
  Vector3d front_vertex = center +  Vector3d(0, max_distance, 0);
  Vector3d back_vertex = center -  Vector3d(0, max_distance, 0);

  std::vector<Vector3d> vertices = {
    top_vertex, bottom_vertex,
    left_vertex, right_vertex,
    front_vertex, back_vertex
  };

  std::chrono::_V2::system_clock::rep t = std::chrono::steady_clock::now().time_since_epoch().count();
  double angle = 2 * M_PI * (static_cast<double>(t % static_cast<int>(time_interval * 1e9)) / (time_interval * 1e9));

  Matrix3d rotation_matrix;
  rotation_matrix =  AngleAxisd(angle,  Vector3d::UnitZ());

  std::vector<Vector3d> rotated_vertices;
  for (const Vector3d& vertex : vertices)
  {
    rotated_vertices.push_back(rotation_matrix * vertex);
  }

  return rotated_vertices;  
}

std::vector<Vector3d> WaypointPublisher::generate_hor_rotating_lines() {
    double R = 0.8;
    double z_rot = 1.5;
    double d = 0.7;
    double x0 = 1.2;

    Vector3d rot_axis(0, 0, z_rot);

    double frequency = 0.1;  // Hz
    double time_interval = 1.0 / frequency;

    Vector3d A1(x0 - 0.0 * d, 0, z_rot + R);
    Vector3d A2(x0 - 1.0 * d, 0, z_rot + R);
    Vector3d A3(x0 - 2.0 * d, 0, z_rot + R);
    Vector3d A4(x0 - 3.0 * d, 0, z_rot + R);

    Vector3d B1(x0 - 0.5 * d, 0, z_rot - R);
    Vector3d B2(x0 - 1.5 * d, 0, z_rot - R);
    Vector3d B3(x0 - 2.5 * d, 0, z_rot - R);
    Vector3d B4(x0 - 3.5 * d, 0, z_rot - R);

    std::vector<Vector3d> vertices = {A1, A2, A3, A4, B1, B2, B3, B4};

    // Translate first
    for (auto& vertex : vertices) {
        vertex -= rot_axis;
    }

    // Rotate
    auto t = std::chrono::system_clock::now();
    double angle = 2 * M_PI * std::chrono::duration<double>(t.time_since_epoch()).count() / time_interval;

    Matrix3d rotation_matrix;
    rotation_matrix << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle);

    for (auto& vertex : vertices) {
        vertex = rotation_matrix * vertex;
    }

    // Translate back
    for (auto& vertex : vertices) {
        vertex += rot_axis;
    }

    return vertices;
}

std::vector<Vector3d> WaypointPublisher::generate_ver_rotating_lines() {
    double R = 0.8;
    double d = 0.7;
    double z0 = 0.3;

    double frequency = 0.05;  // Hz
    double time_interval = 1.0 / frequency;

    Vector3d A1(R, 0, z0 + 0 * d);
    Vector3d A2(R, 0.1, z0 + 1 * d);
    Vector3d A3(R, -0.1, z0 + 2 * d);
    Vector3d A4(R, 0, z0 + 3 * d);

    Vector3d B1(-R, 0, z0 + 0 * d);
    Vector3d B2(-R, 0.1, z0 + 1 * d);
    Vector3d B3(-R, -0.1, z0 + 2 * d);
    Vector3d B4(-R, 0, z0 + 3 * d);

    std::vector<Vector3d> vertices = {A1, A2, A3, A4, B1, B2, B3, B4};

    // Rotate
    auto t = std::chrono::system_clock::now();
    double angle = 2 * M_PI * std::chrono::duration<double>(t.time_since_epoch()).count() / time_interval;

    Matrix3d rotation_matrix;
    rotation_matrix << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;

    for (auto& vertex : vertices) {
        vertex = rotation_matrix * vertex;
    }

    return vertices;
}

std::vector<Vector3d> WaypointPublisher::generate_spiral() {
    double R = 0.5;
    double z0 = 0.25;

    double frequency = 0.2;  // Hz
    double time_interval = 1.0 / frequency;
    
    double dth = 60.0 / 57.3;
    double dh = 0.3;

    Vector3d A(R * cos(0 * dth), R * sin(0 * dth), 0 * dh + z0);
    Vector3d B(R * cos(1 * dth), R * sin(1 * dth), 1 * dh + z0);
    Vector3d C(R * cos(2 * dth), R * sin(2 * dth), 2 * dh + z0);
    Vector3d D(R * cos(3 * dth), R * sin(3 * dth), 3 * dh + z0);
    Vector3d E(R * cos(4 * dth), R * sin(4 * dth), 4 * dh + z0);
    Vector3d F(R * cos(5 * dth), R * sin(5 * dth), 5 * dh + z0);
    Vector3d G(R * cos(6 * dth), R * sin(6 * dth), 6 * dh + z0);
    Vector3d H(R * cos(7 * dth), R * sin(7 * dth), 7 * dh + z0);
    Vector3d I(R * cos(8 * dth), R * sin(8 * dth), 9 * dh + z0);

    std::vector<Vector3d> vertices = {A, B, C, D, E, F, G, H, I};

    // Rotate
    auto t = std::chrono::system_clock::now();
    double angle = 2 * M_PI * std::chrono::duration<double>(t.time_since_epoch()).count() / time_interval;

    Matrix3d rotation_matrix;
    rotation_matrix << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;

    for (auto& vertex : vertices) {
        vertex = rotation_matrix * vertex;
    }

    return vertices;
}

std::vector<Vector3d> WaypointPublisher::generate_smiley()
{
    double frequency = 0.05; // Hz
    double time_interval = 1.0 / frequency;

    Vector3d left_eye(0, 0.5, 1.5);
    Vector3d right_eye(0, -0.5, 1.5);

    Vector3d mouth1(0, -0.8, 1.0);
    Vector3d mouth2(0, -0.5, 0.6);
    Vector3d mouth3(0, -0.2, 0.4);
    Vector3d mouth4(0, 0.2, 0.4);
    Vector3d mouth5(0, 0.5, 0.6);
    Vector3d mouth6(0, 0.8, 1.0);

    std::vector<Vector3d> vertices = {
        left_eye, right_eye, mouth1, mouth2, mouth3, mouth4, mouth5, mouth6
    };

    auto t = std::chrono::system_clock::now();
    std::chrono::duration<double> time_since_epoch = t.time_since_epoch();
    double angle = 2 * M_PI * fmod(time_since_epoch.count(), time_interval) / time_interval;

    Matrix3d rotation_matrix;
    rotation_matrix << std::cos(angle), -std::sin(angle), 0,
                       std::sin(angle), std::cos(angle), 0,
                       0, 0, 1;

    for (auto& vertex : vertices)
    {
        vertex = rotation_matrix * vertex;
    }

    return vertices;
}

std::vector<Vector3d> WaypointPublisher::generate_sinwave()
{
    double x_min = -1.0; // Minimum x-coordinate
    double x_max = 1.0;  // Maximum x-coordinate
    int num_drones_per_line = 4;  // Number of points in the x direction
    int num_lines = 2;  // Number of lines

    // Generate x-coordinates for the grid
    VectorXd x_coordinates = VectorXd::LinSpaced(num_drones_per_line, x_min, x_max);

    std::vector<Vector3d> grid_points;

    double amplitude = 0.4;  // Adjust as needed
    double frequency = 0.8;  // Adjust as needed

    auto t = std::chrono::system_clock::now();
    std::chrono::duration<double> time_since_epoch = t.time_since_epoch();

    for (int line_index = 0; line_index < num_lines; ++line_index)
    {
        double y = (line_index - (num_lines - 1) / 2.0) * 0.75;  // Adjust the spacing as needed
        VectorXd z_coordinates = 1.0 + amplitude * (x_coordinates.array().unaryExpr([&](double x) { return std::sin(frequency * time_since_epoch.count() - x); }));

        for (int i = 0; i < num_drones_per_line; ++i)
        {
            grid_points.emplace_back(x_coordinates(i), y, z_coordinates(i));
        }
    }

    return grid_points;
}

std::vector<Vector3d> WaypointPublisher::generate_landing_test()
{
    int no_drones = 8;
    double spacing = 0.5;
    double height = 1.0;
    Eigen::Vector2d offset = Eigen::Vector2d(0.0, 0.0);
    double frequency = 0.1;  // Hz
    double time_interval = 1.0 / frequency;

    int grid_size = std::ceil(std::sqrt(no_drones));
    std::vector<Vector3d> grid;
    for (int x = 0; x < grid_size; ++x)
    {
        for (int y = 0; y < grid_size; ++y)
        {
            grid.push_back(Vector3d(
                (x - (grid_size - 1) / 2.0) * spacing + offset[0],
                (y - (grid_size - 1) / 2.0) * spacing + offset[1],
                height));
        }
    }

    // Calculate the current angle of rotation
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    double angle = 2 * M_PI * (static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % static_cast<int>(time_interval * 1e9)) / (time_interval * 1e9));

    // Create the rotation matrix
    Matrix3d rotation_matrix;
    rotation_matrix = AngleAxisd(angle, Vector3d::UnitZ());

    // Center of the grid for rotation
    Vector3d center = Vector3d(offset[0], offset[1], height);

    // Rotate all grid points
    std::vector<Vector3d> rotated_grid;
    for (const Vector3d& point : grid) {
        rotated_grid.push_back(rotation_matrix * (point - center) + center);
    }

    //Land 1 in 15 times
    int randomNumber = rand() % 15;
    if (randomNumber >= 14) {
        topic_interface::msg::ControllerCommand msg;
        msg.uri=std::string("all");
        msg.data=std::string("land in place");
        this->command_pub_->publish(msg);
        // self.command_pub.publish(ControllerCommand(uri=uri, data='land in place'))
        // self.command_pub = self.create_publisher(ControllerCommand, 'controller_command', 10)

    }

    return rotated_grid;
}