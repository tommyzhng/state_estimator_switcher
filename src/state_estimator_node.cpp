#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <state_estimator/Mocap.h>
#include <std_srvs/SetBool.h>

// node that reads params to know which estimator to use. Indoor uses mocap, outdoor uses gps from mavros
// param is called indoor_mode

class StateEstimatorNode
{
    public:
    StateEstimatorNode()
    {
        ros::NodeHandle nh("~");

        nh.param("indoor_mode", indoor_mode, true);

        // initialize subscriber
        ros::Subscriber local_position_sub;
        if (indoor_mode)
        {
            std::string mocap_topic = "/mocap/UAV0";
            local_position_sub = nh.subscribe(mocap_topic, 1, &StateEstimatorNode::mocapCallback, this);
        }
        else
        {
            local_position_sub = nh.subscribe("/mavros/local_position/odom", 1, &StateEstimatorNode::localPositionCallback, this);
        }

        // initialize publisher
        std::string state_topic = "/mavros/local_position/odom/UAV0";
        state_pub = nh.advertise<nav_msgs::Odometry>(state_topic, 1);

        // initialize service
        ros::ServiceServer estimator_type_server = nh.advertiseService("/estimator_type", &StateEstimatorNode::estimatorType, this);

        ros::Rate rate(100.0);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    
    }

private:
    ros::Publisher state_pub;
    bool indoor_mode;

    void mocapCallback(const state_estimator::Mocap::ConstPtr &msg)
    {
        nav_msgs::Odometry state;
        state.header = msg->header;

        state.pose.pose.position.x = msg->position[0];
        state.pose.pose.position.y = msg->position[1];
        state.pose.pose.position.z = msg->position[2];

        state.twist.twist.linear.x = msg->velocity[0];
        state.twist.twist.linear.y = msg->velocity[1];
        state.twist.twist.linear.z = msg->velocity[2];

        state.twist.twist.angular.x = msg->angular_velocity[0];
        state.twist.twist.angular.y = msg->angular_velocity[1];
        state.twist.twist.angular.z = msg->angular_velocity[2];

        state.pose.pose.orientation.x = msg->quaternion[1];
        state.pose.pose.orientation.y = msg->quaternion[2];
        state.pose.pose.orientation.z = msg->quaternion[3];
        state.pose.pose.orientation.w = msg->quaternion[0];

        state_pub.publish(state);
    }

    void localPositionCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        state_pub.publish(msg);
    }

    bool estimatorType(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        res.success = (indoor_mode) ? true : false;
        return true;
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimator_node");
    StateEstimatorNode state_estimator_node;

    return 0;
}