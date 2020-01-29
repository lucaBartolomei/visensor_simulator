
#ifndef VISENSOR_SIMULATOR_SIMPLE_WAYPOINT_PLANNER_H
#define VISENSOR_SIMULATOR_SIMPLE_WAYPOINT_PLANNER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visensor_simulator/builtin_planner.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

struct Waypoint {
  double x;            // m
  double y;            // m
  double z;            // m
  double yaw;          // rad
  float gimbal_pitch;  // deg
  Waypoint(
      double x_p, double y_p, double z_p, double yaw_p, float gimbal_pitch_p)
      : x(x_p), y(y_p), z(z_p), yaw(yaw_p), gimbal_pitch(gimbal_pitch_p) {}
};

class TrajectoryPlanner : public BuiltInPlanner {
 public:
  // error in 0.2rad(=11.5deg) and meters- these numbers need to be proportinal
  // to the noise in the odometry used as input.
  TrajectoryPlanner();

  TrajectoryPlanner(double yaw_max_error, double position_max_error);
  ~TrajectoryPlanner() {}

  bool loadConfigurationFromFile(const std::string& project_folder_path) override;

  PlannerStatus getStatus() override;

 private:
  void robotOdometryCallback(const nav_msgs::Odometry& curr_odometry);
  void generateTrajectory();
  void generateGimbalTrajectory();

  bool is_valid_;
  double yaw_max_error_;
  double position_max_error_squared_;
  std::vector<Waypoint> waypoints_;
  PlannerStatus status_;

  ros::NodeHandle nh_;

  ros::Publisher pose_command_pub_;
  ros::Publisher gimbal_command_pub_;
  ros::Publisher path_publisher_;
  ros::Subscriber robot_odometry_sub_;

  mav_trajectory_generation::Trajectory trajectory_with_yaw_;
  mav_trajectory_generation::Trajectory trajectory_gimbal_;
};

#endif
