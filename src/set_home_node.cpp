#include <map>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <applanix_msgs/NavigationSolutionGsof49.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std::string_literals;

class SetHomeNode
{
public:
    SetHomeNode()
        : APXMode(true) // Initialize APXMode to false by default
    {
        home_position_.x = 0.0;
        home_position_.y = 0.0;
        home_position_.z = 0.0;

        current_odom_.pose.pose.position.x = 0.0;
        current_odom_.pose.pose.position.y = 0.0;
        current_odom_.pose.pose.position.z = 0.0;

        diff_position_.pose.pose.position.x = 0.0;
        diff_position_.pose.pose.position.y = 0.0;
        diff_position_.pose.pose.position.z = 0.0;

        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        auto uav_prefix = pnh.param("uav_prefix", ""s);

        ROS_INFO("Starting set home node for UAV %s", uav_prefix.c_str());
        set_home_server_ = nh.advertiseService(uav_prefix + "/state_estimator/override_set_home", &SetHomeNode::setHomeCallback, this);

        // subscribe to the current position in mavros GPS
        subs_["translational_state_source"] = nh.subscribe(uav_prefix + "/mavros/local_position/pose", 1, &SetHomeNode::poseCallback, this);
        subs_["velocity_source"] = nh.subscribe(uav_prefix + "/mavros/local_position/velocity_local", 1, &SetHomeNode::velocityCallback, this);
        subs_["rotational_state_source"] = nh.subscribe(uav_prefix + "/mavros/imu/data", 1, &SetHomeNode::attitudeCallback, this);

        // subscribe to the APX GPS topic
        subs_["apx_gps_source"] = nh.subscribe(uav_prefix + "/lvx_client_node/gsof/49", 1, &SetHomeNode::apxPositionCallback, this);

        // advertise the difference between the home position and the current position
        diff_position_pub_ = nh.advertise<nav_msgs::Odometry>(uav_prefix + "/state_estimator/local_position/odom_adjusted", 1);

        ROS_INFO("Set Home Node Initialized");

        ros::Rate rate(60);

        while (ros::ok())
        {
            computeDifference();
            diff_position_pub_.publish(diff_position_);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::ServiceServer set_home_server_;
    std::map<std::string, ros::Subscriber> subs_;
    ros::Publisher diff_position_pub_;

    geometry_msgs::Point home_position_;
    nav_msgs::Odometry current_odom_;
    nav_msgs::Odometry diff_position_;

    bool APXMode; // Boolean to switch between GPS modes

    // Callback for the original GPS position
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // ROS_INFO("I'm here in the outer loop");

        if (!APXMode)
        {
            currentPositionCallback(msg);
        }
    }

    // Callback for the APX GPS position
    void apxPositionCallback(const applanix_msgs::NavigationSolutionGsof49::ConstPtr &msg)
    {
        if (APXMode)
        {
            // ROS_INFO("I'm here when APXMode = true");
            // LLA coordinates from the message
            double lat = msg->lla.latitude;
            double lon = msg->lla.longitude;
            double alt = msg->lla.altitude;

            // LLA of the origin of ENU
            double ref_lat = 43.781759;
            double ref_lon = -79.466227;
            double ref_alt = 190;

            // Convert LLA to ENU
            GeographicLib::LocalCartesian enu(ref_lat, ref_lon, ref_alt);
            double x, y, z;
            enu.Forward(lat, lon, alt, x, y, z);

            // Set the ENU coordinates in the odometry message
            current_odom_.pose.pose.position.x = x;
            current_odom_.pose.pose.position.y = y;
            current_odom_.pose.pose.position.z = z;

            // Linear velocity
            current_odom_.twist.twist.linear.x = msg->velocity.east;  // E
            current_odom_.twist.twist.linear.y = msg->velocity.north; // N
            current_odom_.twist.twist.linear.z = -msg->velocity.down; // U
        }
    }

    // Handle position update for the original GPS
    void currentPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // ROS_INFO("I'm here when APXMode = false");
        current_odom_.header = msg->header;
        current_odom_.pose.pose.position = msg->pose.position;
    }

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        current_odom_.header = msg->header;
        current_odom_.twist.twist.linear = msg->twist.linear;
    }

    void attitudeCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        current_odom_.header = msg->header;
        current_odom_.pose.pose.orientation = msg->orientation;
        current_odom_.twist.twist.angular = msg->angular_velocity;
    }

    bool setHomeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        home_position_.x = current_odom_.pose.pose.position.x;
        home_position_.y = current_odom_.pose.pose.position.y;
        home_position_.z = current_odom_.pose.pose.position.z;

        ROS_INFO("Home position set to: x: %f, y: %f, z: %f", home_position_.x, home_position_.y, home_position_.z);

        return true;
    }

    void computeDifference()
    {
        diff_position_ = current_odom_; // copy the current config
        diff_position_.pose.pose.position.x = current_odom_.pose.pose.position.x - home_position_.x;
        diff_position_.pose.pose.position.y = current_odom_.pose.pose.position.y - home_position_.y;
        diff_position_.pose.pose.position.z = current_odom_.pose.pose.position.z - home_position_.z;
        diff_position_.twist = current_odom_.twist;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_home_node");
    SetHomeNode set_home_node;
    return 0;
}
