# state_estimator_switcher

A simple ROS Node that acts as an intermediate switcher between motion capture state estimation and gps state estimation from mavros. Used for switching between indoor and outdoor testing for a GUI software (https://github.com/LonghaoQian/slung-payload-delivery-GS-GUI)

Subscribes to:
* /mavros/local_position/odom
* /mocap/UAV0

Publishes to:
* /mavros/local_position/odom/UAV0
