# ros_websocket_bridge package

This package is developed for communicating with non-ROS scripts through websockets.

## stairs_detector node
This node is designed for communicating with stairs_detector python package.
### Requirements
1. [drn_lite python package](https://www.centauro-project.eu:444/centauro/drn_lite.git)
### Setup
1. Clone the repo
2. Run `catkin_make`. Source `setup.bash`
3. Make sure that `drn_lite` package is running on a known ip and port.
4. Run `rosrun ros_websocket_bridge stairs_detector.py --host <host_ip> --port <host_port>` By default `<host_ip>` is localhost and `<host_port>` is 5000
5. You can communicate through this node by calling `detect_stairs` service.
