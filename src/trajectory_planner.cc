#include <fstream>
#include <boost/filesystem.hpp>
#include <mav_msgs/common.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <sensor_msgs/Joy.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

#include "visensor_simulator/trajectory_planner.h"

static constexpr float kDEG_2_RAD = M_PI / 180.0;

static constexpr double k_v_max = 3.0;
static constexpr double k_a_max = 5.0;
static constexpr double k_w_max = 2.5;
static constexpr double k_w_dot_max = 2.0;
static constexpr double k_sampling_dt = 0.5;

double quat2yaw(const geometry_msgs::Quaternion& q) {
  double siny = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return atan2(siny, cosy);
}

double squared_dist(const geometry_msgs::Point& curr_pos, Waypoint waypoint) {
  double dx = curr_pos.x - waypoint.x;
  double dy = curr_pos.y - waypoint.y;
  double dz = curr_pos.z - waypoint.z;
  return ((dx * dx) + (dy * dy) + (dz * dz));
}

TrajectoryPlanner::TrajectoryPlanner()
    : TrajectoryPlanner(0.2, 0.5) {}

TrajectoryPlanner::TrajectoryPlanner(
    double yaw_max_error, double position_max_error)
    : is_valid_(false),
      yaw_max_error_(yaw_max_error),
      position_max_error_squared_(position_max_error * position_max_error) {
  pose_command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  gimbal_command_pub_ =
      nh_.advertise<sensor_msgs::Joy>("command/gimbal_actuators", 1);

  robot_odometry_sub_ = nh_.subscribe(
      "robot_odometry_topic", 1000,
      &TrajectoryPlanner::robotOdometryCallback, this);

  path_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/polynomial_path", 100, true);

  // sample trajectory
  waypoints_.push_back(Waypoint(1, -5, 2, 0, 0));
  waypoints_.push_back(Waypoint(1, 5, 2, 0, 0));
  waypoints_.push_back(Waypoint(2, 5, 2, 0, 0));
  waypoints_.push_back(Waypoint(2, -5, 2, 0, 0));
  waypoints_.push_back(Waypoint(1, -5, 2, 0, 0));
  status_ = STARTING;
}

bool TrajectoryPlanner::loadConfigurationFromFile(
    const std::string& project_folder_path) {
  waypoints_.clear();
  status_ = INVALID;
  boost::filesystem::path project_folder(project_folder_path);
  boost::filesystem::path waypoint_file =
      project_folder / "waypoints.txt";
  ROS_INFO("Reading waypoints file...");

  if (boost::filesystem::exists(waypoint_file)) {
    std::ifstream file(waypoint_file.c_str());
    if (file.is_open()) {
      double x, y, z, yaw;
      float gimbal_pitch;
      char eater;  // eats commas
      while (file >> x >> eater >> y >> eater >> z >> eater >> yaw >> eater >>
             gimbal_pitch) {
        waypoints_.push_back(Waypoint(x, y, z, yaw * kDEG_2_RAD, gimbal_pitch));
        if (file.eof()) {
          break;
        }
      }
      if (waypoints_.size() > 0) {
        status_ = STARTING;
        return true;
      }
      else{
        ROS_ERROR_STREAM( "" << waypoint_file.c_str() <<" file does not have any waypoint. ");
      }

      }else
      {
        ROS_ERROR_STREAM("" << waypoint_file.c_str() <<" file could not be opened");
      }


  }else
    {
      ROS_ERROR_STREAM(
            "the project folder does not have a waypoints.txt file :"
            << waypoint_file.c_str());
    }


  return false;
}


BuiltInPlanner::PlannerStatus TrajectoryPlanner::getStatus() {
  return status_;
}

void TrajectoryPlanner::robotOdometryCallback(
    const nav_msgs::Odometry& curr_odometry) {

  // Check if we have reached the final waypoint
  if(status_ != STARTING) {
    Eigen::Vector3d final_wp_eigen(waypoints_.back().x, waypoints_.back().y,
                                   waypoints_.back().z);
    Eigen::Vector3d current_position(curr_odometry.pose.pose.position.x,
                                     curr_odometry.pose.pose.position.y,
                                     curr_odometry.pose.pose.position.z);
    if((final_wp_eigen - current_position).norm() <
          std::sqrt(position_max_error_squared_)) {
      status_ = COMPLETED;
      ROS_INFO("Completed trajectory.");
    }
  }

  if(status_ == STARTING) {
    // Generate two trajectories: one for the gimbal (just pitch values) and
    // one for the robot (x, y, z, yaw)
    generateGimbalTrajectory();
    generateTrajectory();

    visualization_msgs::MarkerArray markers;
    mav_trajectory_generation::drawMavTrajectory(trajectory_with_yaw_, 1.0,
                                                 "world", &markers);
    path_publisher_.publish(markers);

    // Publish the trajectories to the controller. For the robot trajectory,
    // publish it once. For the gimbal, we need to send the commands one at the
    // time
    mav_msgs::EigenTrajectoryPoint::Vector states;
    mav_trajectory_generation::sampleWholeTrajectory(trajectory_with_yaw_,
                                                     k_sampling_dt, &states);

    trajectory_msgs::MultiDOFJointTrajectoryPtr msg(
          new trajectory_msgs::MultiDOFJointTrajectory);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "world";
    msg->joint_names.push_back("base_link");

    for(auto state : states) {
      trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
      trajectory_point.transforms.resize(1);
      tf::vectorEigenToMsg(state.position_W,
                           trajectory_point.transforms[0].translation);
      trajectory_point.transforms[0].rotation =
          tf::createQuaternionMsgFromYaw(state.getYaw());
      trajectory_point.time_from_start = ros::Duration(
            state.time_from_start_ns * 1e-9);
      msg->points.push_back(trajectory_point);
    }
    pose_command_pub_.publish(msg);

    ROS_ERROR("Gimbal commands are not gonna be sent!");

    // Update flag
    status_ = RUNNING;
  }
}

void TrajectoryPlanner::generateTrajectory() {

  // Step 1: get the yaws
  std::vector<double> yaws;
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    yaws.push_back(waypoints_[i].yaw);
  }

  // Step 2: generate the verticies for the interpolation
  mav_trajectory_generation::Vertex::Vector vertices, vertices_yaw;
  const int dimension = 3;
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::ACCELERATION;
  const int derivative_to_optimize_yaw =
      mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION;

  mav_trajectory_generation::Vertex vertex(dimension), vertex_yaw(1);
  Eigen::Vector3d wp0_eigen(waypoints_.front().x, waypoints_.front().y,
                           waypoints_.front().z);
  vertex.makeStartOrEnd(wp0_eigen, derivative_to_optimize);

  double yaw_old = yaws[0];
  vertex_yaw.makeStartOrEnd(yaws[0], derivative_to_optimize_yaw);

  vertices.push_back(vertex);
  vertices_yaw.push_back(vertex_yaw);

  for (size_t i = 1; i < waypoints_.size() - 1; ++i) {
    mav_trajectory_generation::Vertex vertex_m(dimension), vertex_yaw_m(1);
    Eigen::Vector3d wp_eigen(waypoints_[i].x, waypoints_[i].y, waypoints_[i].z);
    vertex_m.addConstraint(
        mav_trajectory_generation::derivative_order::POSITION, wp_eigen);

    if (std::abs(yaws[i] + 2 * M_PI - yaw_old) < std::abs(yaws[i] - yaw_old)) {
      yaw_old = yaws[i] + 2 * M_PI;
    } else if (std::abs(yaws[i] - 2 * M_PI - yaw_old) <
               std::abs(yaws[i] - yaw_old)) {
      yaw_old = yaws[i] - 2 * M_PI;
    } else {
      yaw_old = yaws[i];
    }

    vertex_yaw_m.addConstraint(
        mav_trajectory_generation::derivative_order::ORIENTATION, yaw_old);
    vertices.push_back(vertex_m);
    vertices_yaw.push_back(vertex_yaw_m);
  }

  Eigen::Vector3d wp_eigen(waypoints_.back().x, waypoints_.back().y,
                           waypoints_.back().z);
  vertex.makeStartOrEnd(wp_eigen, derivative_to_optimize);
  if (std::abs(yaws.back() + 2 * M_PI - yaw_old) <
      std::abs(yaws.back() - yaw_old)) {
    yaw_old = yaws.back() + 2 * M_PI;
  } else if (std::abs(yaws.back() - 2 * M_PI - yaw_old) <
             std::abs(yaws.back() - yaw_old)) {
    yaw_old = yaws.back() - 2 * M_PI;
  } else {
    yaw_old = yaws.back();
  }
  vertex_yaw.makeStartOrEnd(yaw_old, derivative_to_optimize_yaw);
  vertices.push_back(vertex);
  vertices_yaw.push_back(vertex_yaw);

  // Step 3: obtain interpolated trajectory
  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(vertices, k_v_max,
                                                      k_a_max);
  std::vector<double> segment_times_yaw{
      mav_trajectory_generation::estimateSegmentTimes(vertices_yaw, k_w_max,
                                                      k_w_dot_max)};

  for (int i = 0; i < segment_times.size(); ++i) {
    if (segment_times_yaw[i] > segment_times[i])
      segment_times[i] = segment_times_yaw[i];

    if (segment_times[i] < k_sampling_dt)
      segment_times[i] = k_sampling_dt;
  }

  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  mav_trajectory_generation::PolynomialOptimization<N> opt_yaw(1);
  opt_yaw.setupFromVertices(vertices_yaw, segment_times,
                            derivative_to_optimize_yaw);
  opt_yaw.solveLinear();

  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory trajectory_yaw;
  opt.getTrajectory(&trajectory);
  opt_yaw.getTrajectory(&trajectory_yaw);
  trajectory.getTrajectoryWithAppendedDimension(trajectory_yaw,
                                                &trajectory_with_yaw_);
}

void TrajectoryPlanner::generateGimbalTrajectory() {
  // Step 1: get the yaws
  std::vector<double> pitchs;
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    pitchs.push_back(waypoints_[i].gimbal_pitch);
  }

  // Step 2: generate the verticies for the interpolation
  mav_trajectory_generation::Vertex::Vector vertices_gimbal;
  const int dimension = 1;
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION;

  mav_trajectory_generation::Vertex vertex_gimbal(dimension);
  double pitch_old = pitchs[0];
  vertex_gimbal.makeStartOrEnd(pitchs[0], derivative_to_optimize);
  vertices_gimbal.push_back(vertex_gimbal);

  for (size_t i = 1; i < waypoints_.size() - 1; ++i) {
    mav_trajectory_generation::Vertex vertex_pitch_m(dimension);

    if (std::abs(pitchs[i] + 2 * M_PI - pitch_old) < std::abs(pitchs[i] - pitch_old)) {
      pitch_old = pitchs[i] + 2 * M_PI;
    } else if (std::abs(pitchs[i] - 2 * M_PI - pitch_old) <
               std::abs(pitchs[i] - pitch_old)) {
      pitch_old = pitchs[i] - 2 * M_PI;
    } else {
      pitch_old = pitchs[i];
    }

    vertex_pitch_m.addConstraint(
        mav_trajectory_generation::derivative_order::ORIENTATION, pitch_old);
    vertices_gimbal.push_back(vertex_pitch_m);
  }
  if (std::abs(pitchs.back() + 2 * M_PI - pitch_old) <
      std::abs(pitchs.back() - pitch_old)) {
    pitch_old = pitchs.back() + 2 * M_PI;
  } else if (std::abs(pitchs.back() - 2 * M_PI - pitch_old) <
             std::abs(pitchs.back() - pitch_old)) {
    pitch_old = pitchs.back() - 2 * M_PI;
  } else {
    pitch_old = pitchs.back();
  }

  mav_trajectory_generation::Vertex vertex_gimbal_end(dimension);
  vertex_gimbal_end.makeStartOrEnd(pitch_old, derivative_to_optimize);
  vertices_gimbal.push_back(vertex_gimbal);

  // Step 3: obtain interpolated trajectory
  std::vector<double> segment_times_pitch{
      mav_trajectory_generation::estimateSegmentTimes(vertices_gimbal, k_w_max,
                                                      k_w_dot_max)};

  for (int i = 0; i < segment_times_pitch.size(); ++i) {
    if (segment_times_pitch[i] < k_sampling_dt)
      segment_times_pitch[i] = k_sampling_dt;
  }

  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt_pitch(dimension);
  opt_pitch.setupFromVertices(vertices_gimbal, segment_times_pitch,
                            derivative_to_optimize);
  opt_pitch.solveLinear();
  opt_pitch.getTrajectory(&trajectory_gimbal_);
}
