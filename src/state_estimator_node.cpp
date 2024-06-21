#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <state_estimator/Mocap.h>
#include <std_msgs/Bool.h>

// node that reads params to know which estimator to use. Indoor uses mocap, outdoor uses gps from mavros
// param is called indoorMode

class StateEstimatorNode
{
    public:
    StateEstimatorNode()
    {
        ros::NodeHandle nh("~");

        nh.param("indoorMode", indoorMode, true);

        // initialize subscriber
        ros::Subscriber localPositionSub;
        if (indoorMode)
        {
            std::string mocapTopic = "/mocap/UAV0";
            localPositionSub = nh.subscribe(mocapTopic, 1, &StateEstimatorNode::mocapCallback, this);
        }
        else
        {
            localPositionSub = nh.subscribe("/mavros/local_position/odom", 1, &StateEstimatorNode::localPositionCallback, this);
        }

        // initialize publisher
        std::string stateTopic = "/mavros/local_position/odom/UAV0";
        statePub = nh.advertise<nav_msgs::Odometry>(stateTopic, 1);

        // initialize service
        estimatorTypePub = nh.advertise<std_msgs::Bool>("/estimator_type", 1);

        ros::Rate rate(100.0);
        while (ros::ok())
        {
            ros::spinOnce();
            CheckEstimator();

            rate.sleep();
        }

    }

    void CheckEstimator(void)
    {
        if (loopCounter >= loopThreshold)
        {
            loopCounter = 0;
            std_msgs::Bool success;
            success.data = (indoorMode) ? true : false;
            estimatorTypePub.publish(success);
        }
        loopCounter++;
    }

    void SetMocapFlag()
    {

    }

private:
    ros::Publisher statePub;
    ros::Publisher estimatorTypePub;
    bool indoorMode;
    uint64_t loopCounter{0};
    uint64_t loopThreshold{20};
    
    struct mocapWatchdogState {
        bool mocapRecieved{false};
        uint32_t mocapTimeoutTime{10};
        
    };

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

        statePub.publish(state);
    }

    void localPositionCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        statePub.publish(msg);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimator_node");
    StateEstimatorNode stateEstimatorNode;

    return 0;
}