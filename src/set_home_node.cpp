#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <string>

#include "Eigen/Dense"
#include "mavros_msgs/CommandInt.h"
#include "ros/forwards.h"
#include "tf2_eigen/tf2_eigen.h"

using namespace std::string_literals;

class SetHomeNode {
 public:
  SetHomeNode() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    auto uav_prefix = pnh.param("uav_prefix", ""s);

    ROS_INFO("Starting set home node for UAV %s", uav_prefix.c_str());
    set_home_server_ =
        nh.advertiseService(uav_prefix + "/state_estimator/override_set_home",
                            &SetHomeNode::setHomeCallback, this);

    // subscribe to the current position in mavros gps
    subs_["translational_state_source"] =
        nh.subscribe(uav_prefix + "/mavros/local_position/pose", 1,
                     &SetHomeNode::currentPositionCallback, this);
    subs_["velocity_source"] =
        nh.subscribe(uav_prefix + "/mavros/local_position/velocity_local", 1,
                     &SetHomeNode::velocityCallback, this);
    subs_["rotational_state_source"] =
        nh.subscribe(uav_prefix + "/mavros/imu/data", 1,
                     &SetHomeNode::attitudeCallback, this);

    // advertise the difference between the home position and the current position
    diff_position_pub_ = nh.advertise<nav_msgs::Odometry>(
        uav_prefix + "/state_estimator/local_position/odom_adjusted", 1);

    ROS_INFO("Set Home Node Initialized");

    pub_loop_ = nh.createTimer(60.0, &SetHomeNode::publishLoop, this);
  }

 private:
  ros::ServiceServer set_home_server_;
  std::map<std::string, ros::Subscriber> subs_;
  ros::Publisher diff_position_pub_;

  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d linear_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_{Eigen::Vector3d::Zero()};

  Eigen::Vector3d current_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d home_position_{Eigen::Vector3d::Zero()};
  nav_msgs::Odometry diff_position_;
  ros::Timer pub_loop_;
  void publishLoop(const ros::TimerEvent &_) {
    computeDifference();
    diff_position_pub_.publish(diff_position_);
  }
  bool setHomeCallback(mavros_msgs::CommandInt::Request &req,
                       mavros_msgs::CommandInt::Response &res) {
    current_position_ = position_;

    home_position_ << req.param1, req.param2, req.param3;

    ROS_INFO("Home position set to: x: %f, y: %f, z: %f", current_position_.x(),
             current_position_.y(), current_position_.z());

    return true;
  }

  void currentPositionCallback(
      const geometry_msgs::PoseStamped::ConstPtr &msg) {
    tf2::fromMsg(msg->pose.position, position_);
  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    tf2::fromMsg(msg->twist.linear, linear_velocity_);
  }

  void attitudeCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf2::fromMsg(msg->orientation, attitude_);
    tf2::fromMsg(msg->angular_velocity, angular_velocity_);
  }

  void computeDifference() {
    diff_position_.header.stamp = ros::Time::now();
    const Eigen::Vector3d offset = home_position_ - current_position_;
    const Eigen::Vector3d actual_position = position_ + offset;
    diff_position_.pose.pose.position = tf2::toMsg(actual_position);
    diff_position_.pose.pose.orientation = tf2::toMsg(attitude_);
    tf2::toMsg(linear_velocity_, diff_position_.twist.twist.linear);
    tf2::toMsg(angular_velocity_, diff_position_.twist.twist.angular);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "set_home_node");
  SetHomeNode set_home_node;
  ros::spin();
  return 0;
}
