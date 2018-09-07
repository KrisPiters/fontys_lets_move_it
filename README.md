# fontys_lets_move_it

A metapackage for use within the Fontys Lets Move It project.

## turtlebot_fleet_bringup

A modified version of [turtlebot_bringup](http://wiki.ros.org/turtlebot_bringup).

Modifications have been made to launch files to support a 'prefix' argument for [namespacing](http://wiki.ros.org/Names) nodes, topics and TF frame prefixes.


## turtlebot_fleet_nav_launch

A modified version of [turtlebot_navigation](http://wiki.ros.org/turtlebot_navigation).

Modifications have been made to launch files to support a 'prefix' argument for [namespacing](http://wiki.ros.org/Names) nodes, topics and TF frame prefixes.


## Usage

### Prequisites
- Ubuntu 16.04
- ROS Kinectic
- Git installed
- All turtlebot packages installed
- Clone this package to your machines' catkin workspace
  ```bash
  roscd
  cd ../src
  git clone https://github.com/KrisPiters/fontys_lets_move_it.git
  ```

### Setup
- Configure you network for use with multiple machines [1](http://wiki.ros.org/ROS/NetworkSetup) [2](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) (fontyswpa and eduroam don't support hostname resolving so use ip addresses for *ROS_MASTER_URI*, *ROS_IP*, *ROS_HOSTNAME*)
- Create and save a map ([Example Turtlebot Tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/indigo/Build%20a%20map%20with%20SLAM)). This is the map that will be shared to all Turtlebots in the fleet for navigation and localization by use of [move_base](http://wiki.ros.org/move_base) and [amcl](http://wiki.ros.org/amcl) ([Example Tutorial for a single Turtlebot](http://wiki.ros.org/turtlebot_navigation/Tutorials/indigo/Autonomously%20navigate%20in%20a%20known%20map)).
  - Do note that the starting pose of the robot, when starting the mapping process, will be the default initial pose of the robot within the map when launching amcl_move_base from *turtlebot_fleet_nav_launch*. So it might be useful to mark it, for easy re-use later on without having to set an initial pose for amcl.
  - Make sure to save the map to machine that will run the map_server node
- On the machine that will run your map_server
  - Update the network settings, making sure ROS_MASTER_URI is set to that machine's ip address.
  - Start a roscore
  - Start the map_server
    ```bash
    rosrun map_server map_server location/of/yourmap.yaml
    ```
- On a Turtlebot Machine
  - Its easiest to physically place the robot in the same pose the robot was in when starting the mapping proces
  - Update the machine' s network settings, making sure ROS_MASTER_URI is set to match the ip address of the machine running the roscore.
  - Launch *minimal_prefix.launch* with the value of the prefix argument set to a unique name eg. robot_0, robot_1.
    ```bash
    roslaunch turtlebot_fleet_bringup minimal_prefix.launch prefix:=robot_0
    ```
  - Launch navigation and localization with the same prefix
    ```bash
    roslaunch turtlebot_fleet_nav_launch amcl_move_base_prefix.launch prefix:=robot_0
    ```
- On any machine with gui
  - An example rviz config can be found in *fontys_lets_move_it/turtlebot_fleet_nav_launch/rviz* it's configured for three robots using 'robot_x' naming.
  - A navigaton goal can be set with the *2D nav goal tool*.
  - Change the 2D nav goal topic in tool properties to publish the goal to a different topic, eg */robot_0/move_base_simple/goal* to */robot_1/move_base_simple/goal* to use the 2D Nav tool for robot_1 instead of robot_0
  


  
 
