/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).
   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
*/
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "visualization_msgs/Marker.h"
#include <kdl/frames.hpp>

using std::string;

class PurePursuit
{
  public:
    PurePursuit();
    // Generate the command for the vehicle according to the current position and the waypoints
    void cmd_generator(nav_msgs::Odometry odom);
    // Listen to the waypoints topic
    void waypoints_listener(nav_msgs::Path path);
    // Transform the pose to the base_link
    KDL::Frame trans2base(const geometry_msgs::Pose& pose, const geometry_msgs::Transform& tf);
    // Eucledian distance computation
    template<typename T1, typename T2>
    double distance(T1 pt1, T2 pt2)
    {
      return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
    }
    // Ros_spin.
    void run();
    
  private:
    // Parameters
    double wheel_base_;
    double lookahead_distance_, position_tolerance_;
    double v_max_, v_, w_max_;
    double delta_, delta_vel_, acc_, jerk_, delta_max_;
    int idx_memory;
    unsigned idx_;
    bool goal_reached_, path_loaded_;

    nav_msgs::Path path_;
    geometry_msgs::Twist cmd_vel_;
    ackermann_msgs::AckermannDriveStamped cmd_acker_;
    visualization_msgs::Marker lookahead_marker_;
    
    // ROS
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber sub_odom_, sub_path_;
    ros::Publisher pub_vel_, pub_acker_, pub_marker_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped lookahead_;
    string map_frame_id_, robot_frame_id_, lookahead_frame_id_, acker_frame_id_;

};

PurePursuit::PurePursuit() : lookahead_distance_(1.0), v_max_(0.1), v_(v_max_), w_max_(1.0), position_tolerance_(0.1), idx_(0),
                             goal_reached_(false), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("base_link"), lookahead_frame_id_("lookahead")
{
  // Get parameters from the parameter server
  nh_private_.getParam("wheelbase", wheel_base_);
  nh_private_.getParam("lookahead_distance", lookahead_distance_);
  //nh_private_.getParam("linear_velocity", v_, 0.1);
  nh_private_.getParam("max_rotational_velocity", w_max_);
  nh_private_.getParam("position_tolerance", position_tolerance_);
  nh_private_.getParam("steering_angle_velocity", delta_vel_);
  nh_private_.getParam("acceleration", acc_);
  nh_private_.getParam("jerk", jerk_);
  nh_private_.getParam("steering_angle_limit", delta_max_);
  nh_private_.getParam("map_frame_id", map_frame_id_);
  nh_private_.getParam("robot_frame_id", robot_frame_id_);
  nh_private_.getParam("lookahead_frame_id", lookahead_frame_id_);
  nh_private_.getParam("ackermann_frame_id", acker_frame_id_);

  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;

  cmd_acker_.header.frame_id = acker_frame_id_;
  cmd_acker_.drive.steering_angle_velocity = delta_vel_;
  cmd_acker_.drive.acceleration = acc_;
  cmd_acker_.drive.jerk = jerk_;

  idx_memory = 0;
  path_loaded_ = false;
  
  sub_path_ = nh_.subscribe("/waypoints", 1, &PurePursuit::waypoints_listener, this);
  sub_odom_ = nh_.subscribe("/odom", 1, &PurePursuit::cmd_generator, this);

  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_acker_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_acker", 1);
  pub_marker_ = nh_.advertise<visualization_msgs::Marker>("lookahead", 1);

}

void PurePursuit::cmd_generator(nav_msgs::Odometry odom)
{
  if (path_loaded_)
  {
    // Get the current pose
    geometry_msgs::TransformStamped tf;
    try
    {
      tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
      // Detetmine the waypoint to track on the basis of 1) current_pose 2) waypoints_info 3) lookahead_distance
      for (idx_=idx_memory; idx_ < path_.poses.size(); idx_++)
      {
        if (distance(path_.poses[idx_].pose.position, tf.transform.translation) > lookahead_distance_)
        {
          KDL::Frame pose_offset = trans2base(path_.poses[idx_].pose, tf.transform);
          lookahead_.transform.translation.x = pose_offset.p.x();
          lookahead_.transform.translation.y = pose_offset.p.y();
          lookahead_.transform.translation.z = pose_offset.p.z();
          pose_offset.M.GetQuaternion(lookahead_.transform.rotation.x, lookahead_.transform.rotation.y,
                                      lookahead_.transform.rotation.z, lookahead_.transform.rotation.w);
          idx_memory = idx_;
          break;
        }
      }

      // If approach the goal (last waypoint)
      if (!path_.poses.empty() && idx_ >= path_.poses.size())
      {
        KDL::Frame goal_offset = trans2base(path_.poses.back().pose, tf.transform);

        // Reach the goal
        if (fabs(goal_offset.p.x()) <= position_tolerance_)
        {
          goal_reached_ = true;
          path_ = nav_msgs::Path(); // Reset the path
        }
        // Not meet the position tolerance: extend the lookahead distance beyond the goal
        else
        {
          // Find the intersection between the circle of radius(lookahead_distance) centered at the current pose
          // and the line defined by the last waypoint
          double roll, pitch, yaw;
          goal_offset.M.GetRPY(roll, pitch, yaw);
          double k_end = tan(yaw); // Slope of line defined by the last waypoint
          double l_end = goal_offset.p.y() - k_end * goal_offset.p.x();
          double a = 1 + k_end * k_end;
          double b = 2 * l_end;
          double c = l_end * l_end - lookahead_distance_ * lookahead_distance_;
          double D = sqrt(b*b - 4*a*c);
          double x_ld = (-b + copysign(D,v_)) / (2*a);
          double y_ld = k_end * x_ld + l_end;
          
          lookahead_.transform.translation.x = x_ld;
          lookahead_.transform.translation.y = y_ld;
          lookahead_.transform.translation.z = goal_offset.p.z();
          goal_offset.M.GetQuaternion(lookahead_.transform.rotation.x, lookahead_.transform.rotation.y,
                                      lookahead_.transform.rotation.z, lookahead_.transform.rotation.w);
        }
      }

      // Waypoint follower
      if (!goal_reached_)
      {
        v_ = copysign(v_max_, v_);
        
        double lateral_offset = lookahead_.transform.translation.y;
        cmd_vel_.angular.z = std::min(2*v_/lookahead_distance_*lookahead_distance_*lateral_offset, w_max_);

        // Desired Ackermann steering_angle
        cmd_acker_.drive.steering_angle = std::min(atan2(2*lateral_offset*wheel_base_, lookahead_distance_*lookahead_distance_), delta_max_);
        
        // Linear velocity
        cmd_vel_.linear.x = v_;
        cmd_acker_.drive.speed = v_;
        cmd_acker_.header.stamp = ros::Time::now();
      }
      // Reach the goal: stop the vehicle
      else
      {
        cmd_vel_.linear.x = 0.00;
        cmd_vel_.angular.z = 0.00;

        cmd_acker_.header.stamp = ros::Time::now();
        cmd_acker_.drive.steering_angle = 0.00;
        cmd_acker_.drive.speed = 0.00;
      }

      // Publish the lookahead target transform.
      lookahead_.header.stamp = ros::Time::now();
      tf_broadcaster_.sendTransform(lookahead_);
      // Publish the velocity command
      pub_vel_.publish(cmd_vel_);
      // Publish the ackerman_steering command
      pub_acker_.publish(cmd_acker_);
      // Publish the lookahead_marker for visualization
      lookahead_marker_.header.frame_id = "world";
      lookahead_marker_.header.stamp = ros::Time::now();
      lookahead_marker_.type = visualization_msgs::Marker::SPHERE;
      lookahead_marker_.action = visualization_msgs::Marker::ADD;
      lookahead_marker_.scale.x = 1;
      lookahead_marker_.scale.y = 1;
      lookahead_marker_.scale.z = 1;
      lookahead_marker_.pose.orientation.x = 0.0;
      lookahead_marker_.pose.orientation.y = 0.0;
      lookahead_marker_.pose.orientation.z = 0.0;
      lookahead_marker_.pose.orientation.w = 1.0;
      lookahead_marker_.color.a = 1.0;
      if (!goal_reached_)
      {
        lookahead_marker_.id = idx_;
        lookahead_marker_.pose.position.x = path_.poses[idx_].pose.position.x;
        lookahead_marker_.pose.position.y = path_.poses[idx_].pose.position.y;
        lookahead_marker_.pose.position.z = path_.poses[idx_].pose.position.z;
        lookahead_marker_.color.r = 0.0;
        lookahead_marker_.color.g = 1.0;
        lookahead_marker_.color.b = 0.0;
        pub_marker_.publish(lookahead_marker_);
      }
      else
      {
        lookahead_marker_.id = idx_memory;
        idx_memory += 1;
        lookahead_marker_.pose.position.x = tf.transform.translation.x;
        lookahead_marker_.pose.position.y = tf.transform.translation.y;
        lookahead_marker_.pose.position.z = tf.transform.translation.z;
        lookahead_marker_.color.r = 1.0;
        lookahead_marker_.color.g = 0.0;
        lookahead_marker_.color.b = 0.0;
        if (idx_memory%5 == 0)
        {
          pub_marker_.publish(lookahead_marker_); 
        }
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM(ex.what());
    }
  }
}

void PurePursuit::waypoints_listener(nav_msgs::Path new_path)
{ 
  if (new_path.header.frame_id == map_frame_id_)
  {
    path_ = new_path;
    idx_ = 0;
    if (new_path.poses.size() > 0)
    {
      std::cout << "Received Waypoints" << std::endl;
      path_loaded_ = true;
    }
    else
    {
      ROS_WARN_STREAM("Received empty waypoint!");
    }
  }
  else
  {
    ROS_WARN_STREAM("The waypoints must be published in the " << map_frame_id_ << " frame! Ignoring path in " << new_path.header.frame_id << " frame!");
  }
}

KDL::Frame PurePursuit::trans2base(const geometry_msgs::Pose& pose, const geometry_msgs::Transform& tf)
{
  // Pose in map
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                        KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
  // base_link in map
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w),
                      KDL::Vector(tf.translation.x, tf.translation.y, tf.translation.z));
                      
  return F_map_tf.Inverse()*F_map_pose;
}

void PurePursuit::run()
{
  ros::spin();
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "pure_pursuit");

  PurePursuit controller;
  controller.run();

  return 0;
}
