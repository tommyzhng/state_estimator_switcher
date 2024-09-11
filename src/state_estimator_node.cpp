#include <ros/ros.h>

#include "state_estimator/state_estimator.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_estimator_node");
  ros::NodeHandle nh;
  ros::Rate rate(100.0);
  fsc::StateEstimatorNode stateEstimator(nh);
  while (ros::ok()) {
    ros::spinOnce();
    stateEstimator.Update();
    rate.sleep();
  }
  return 0;
}
