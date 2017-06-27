# The youbot_base_driver package

This is a single-node, ROS wrapper of `youbot_driver` for use only with mobile base (no arm). 

## The youbot_base_driver_node 
The node handles communication with internal youBot drivers.

### Subscribers
- `controls` (`geometry_msgs/Twist`) - control signals to be executed, described with relation to local frame id.

### Publishers
- `odom` (`nav_msgs/Odometry`) - pose and velocity estimated based on odometry of youBot.

### Parameters
- `publish_tf` (`bool`, default: `true`) - decide whether to publish transformation (tf) or not,
- `loop_rate` (`double`, default: `100.0`) - rate of the main loop in Hz,
- `parent_frame_id` (`string`, default: `odom`) - name of the coordinate frame with relate to which the odometry position is described,
- `child_frame_id` (`string`, default: `base`) - name of the coordinate frame attached to the robot base,
- `config_file_path` (`string`, default: `config`) - path to the folder containing `youbot_driver` config files,
- `config_file_name` (`string`, default: `youbot-base`) - name of the base config file without extension.

### Issues
- The nodelet version of the driver (youbot_base_driver/YoubotBaseDriver) does not currently work. Use standard node instead.
- After compilation the executable needs access to the ethernet. Use: `sudo setcap cap_net_raw+ep <path_to_executable_folder>/youbot_base_driver_node`

