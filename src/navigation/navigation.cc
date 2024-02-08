//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <cmath>
#include <iostream>
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::max;
using std::min;
using std::tuple;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"
  // std::cout << point_cloud_.size() << "\n";
  float new_cur, new_distance;
  std::tie(new_cur, new_distance) = GetCurvature(point_cloud_);
  float new_vel = GetVelocity(new_distance);

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = new_cur;
  drive_msg_.velocity = new_vel;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

float GetDistance(const Vector2f& point1, const Vector2f& point2) {
  return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

tuple<float, float> Navigation::GetCurvature(const vector<Vector2f>& point_cloud) {
  const float car_length = 0.535;
  const float car_width = 0.281;
  const float wheel_base = 0.324;
  // const float max_curvature = 1.0;

  const float safety_margin = 0.1;

  const float base_link_to_side = car_width / 2;
  const float base_link_to_front = (car_length + wheel_base) / 2;

  float goal = 5;
  Vector2f base_link(0, 0);

  for (Vector2f point: point_cloud) {
    visualization::DrawCross(point, 0.01, 0xFF0000, local_viz_msg_);
  }

  // vector<float> curvature_candidates;
  // for (float i = -max_curvature; i <= max_curvature; i += 0.1) {
  //   curvature_candidates.push_back(i);
  // }

  // const vector<float> curvature_candidates {-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0};
  const vector<float> curvature_candidates {-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  vector<float> scores;
  vector<float> free_path_lengths;
  for (float curvature_candidate: curvature_candidates) {
    Vector2f point_of_interest;
    if (curvature_candidate == 0) {
      point_of_interest << 5 + base_link_to_front + safety_margin, 0;
      float free_path_length = goal;
      float clearance = 0.5;
      // float distance_to_goal = goal;
      for (Vector2f point: point_cloud) {
        if (point[0] <= 0) {
          continue;
        }
        if (abs(point[1]) < base_link_to_side + safety_margin) {  // hit front
          if (point[0] - base_link_to_front - safety_margin < free_path_length) {
            point_of_interest << point[0], 0;
          }
          free_path_length = min(free_path_length, point[0] - base_link_to_front - safety_margin);
          clearance = 0;
        } else {
          clearance = min(clearance, abs(point[1]) - base_link_to_side - safety_margin);
          // distance_to_goal = min(distance_to_goal, abs(goal[1]));
        }
      }
      scores.push_back(free_path_length + 2 * clearance);
      free_path_lengths.push_back(free_path_length);

      visualization::DrawCross(point_of_interest, 0.1, 0xFF0000, local_viz_msg_);
      visualization::DrawLine(base_link, point_of_interest, 0xFF0000, local_viz_msg_);
      continue;
    }
    float turning_radius = abs(1 / curvature_candidate);
    // float steering_angle = atan(wheel_base / turning_radius);
    float min_radius = turning_radius - base_link_to_side;
    Vector2f turning_center;
    Vector2f front_close;
    Vector2f front_far;
    if (curvature_candidate < 0) {
      turning_center << 0.0, -turning_radius;
      front_close << base_link_to_front, -base_link_to_side;
      front_far << base_link_to_front, base_link_to_side;
    } else {
      turning_center << 0.0, turning_radius;
      front_close << base_link_to_front, base_link_to_side;
      front_far << base_link_to_front, -base_link_to_side;
    }
    float max_radius = GetDistance(turning_center, front_far);
    float front_radius = GetDistance(turning_center, front_close);

    float max_free_path_angle = atan(goal / turning_radius);
    float angle_of_interest = max_free_path_angle;
    float free_path_length = min(max_free_path_angle * turning_radius, goal);
    // float distance_to_goal = get_distance(goal, turning_center) - turning_radius;
    float clearance = 0.5;
    if (turning_radius > front_radius) {
      max_free_path_angle += asin(base_link_to_front / turning_radius);
    } else {
      max_free_path_angle += acos((turning_radius - base_link_to_side) / turning_radius);
    }
    if (curvature_candidate > 0) {
      point_of_interest << sin(max_free_path_angle) * turning_radius, (1-cos(max_free_path_angle))*turning_radius;
    } else {
      point_of_interest << sin(max_free_path_angle) * turning_radius, (cos(max_free_path_angle)-1)*turning_radius;
    }

    for (Vector2f point: point_cloud) {
      if (point[0] <= 0) {
        continue;
      }
      if (curvature_candidate < 0 && point[1] < turning_center[1]) {
        continue;
      }
      if (curvature_candidate > 0 && point[1] > turning_center[1]) {
        continue;
      }
      float angle = atan(point[0] / abs(turning_center[1] - point[1]));
      if (angle >= max_free_path_angle) {
        continue;
      }
      float distance_to_center = GetDistance(point, turning_center);
      if (distance_to_center < min_radius - safety_margin) {
        clearance = min(clearance, min_radius - distance_to_center - safety_margin);
      } else if (distance_to_center > max_radius + safety_margin) {
        clearance = min(clearance, distance_to_center - max_radius - safety_margin);
      } else {
        float hit_point_angle;
        if (distance_to_center > front_radius) {  // hit front
          hit_point_angle = asin(base_link_to_front / distance_to_center);
        } else {  // hit side
          hit_point_angle = acos(min_radius / distance_to_center);
        }
        if (angle > hit_point_angle) {
          if ((angle - hit_point_angle) * turning_radius > free_path_length) {
            angle_of_interest = angle;
            point_of_interest << point[0], point[1];
            clearance = 0;
          }
          free_path_length = min(free_path_length, (angle - hit_point_angle) * turning_radius);

        }
        // else {
        //   free_path_length = 0;
        // }
      }
    }
    scores.push_back(free_path_length + 2 * clearance);
    free_path_lengths.push_back(free_path_length);

    visualization::DrawCross(point_of_interest, 0.1, 0xFF0000, local_viz_msg_);
    if (curvature_candidate > 0) {
      visualization::DrawArc(turning_center, turning_radius, -atan(1)*2, angle_of_interest-atan(1)*2, 0xFF0000, local_viz_msg_);
    } else {
      visualization::DrawArc(turning_center, turning_radius, atan(1)*2-angle_of_interest, atan(1)*2, 0xFF0000, local_viz_msg_);
    }
  }

  float max_score = 0;
  float curvature = 0;
  float distance_to_goal = 0;
  for (size_t i = 0; i < scores.size(); i++) {
    cout << "Cur: " << curvature_candidates[i] << "\t Dist: "<< free_path_lengths[i] << "\n";
    if (scores[i] > max_score) {
      max_score = scores[i];
      curvature = curvature_candidates[i];
      distance_to_goal = free_path_lengths[i];
    }
  }
  cout << "\n";
  return std::make_tuple(curvature, distance_to_goal);
}

float Navigation::GetVelocity(float distance_to_goal) {  // TOC
  const float max_acc = 4;
  const float max_dec = 4.0;
  const float max_vel = 1;
  const float update_interval = 0.05;
  const float latency = 0.1;

  float x_3 = pow(robot_vel_[0], 2) / (2 * max_dec) + latency * robot_vel_[0];
  x_3 += robot_vel_[0] * latency;
  if (x_3 >= distance_to_goal) {  // Deceleration
    return robot_vel_[0] - max_dec * update_interval;
  } else if (robot_vel_[0] >= max_vel) {  // Cruise
    return max(robot_vel_[0] - max_dec * update_interval, max_vel);
  } else {
    return min(robot_vel_[0] + max_acc * update_interval, max_vel);
  }
  return 0;
}

}  // namespace navigation
