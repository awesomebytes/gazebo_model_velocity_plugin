# gazebo_model_velocity_plugin

A plugin that does the same (but all together) than:

* gazebo_ros_p3d: Publishes odometry, but it supports gaussian noise for XY & Yaw separately (and it doesn't add the noise when the robot is stopped). It allows the odom frame to be `odom` (which `gazebo_ros_p3d` doesn't for some reason).
* ros_planar_move: It moves the robot by sending velocities in Gazebo in a Twist topic, but it also implements velocity, acceleration and jerk limits to better behave like a real robot. It also stops the robot if no commands are sent for a while. Kind of a diff_drive_controller but hacky.


# Try it


# Config
Looks like:
```xml
<gazebo>
  <plugin name="base_drive_controller" filename="libgazebo_ros_model_velocity.so">
  <robotNamespace>boxbot</robotNamespace>
  <commandTopic>cmd_vel</commandTopic>
  <!-- this topic will output the actual velocity sent in Gazebo (after applying limits) -->
  <outputVelocityTopic>output_vel</outputVelocityTopic>
  <!-- How often do we send velocity commands in Gazebo -->
  <updateRate>50.0</updateRate>
  <!-- After how long of not hearing a new command in commandTopic do we send a 0 to the robot -->
  <commandTimeout>0.5</commandTimeout>
  <!-- Publish odometry as needed -->
  <odometryTopic>odom</odometryTopic>
  <odometryFrame>odom</odometryFrame>
  <odometryRate>20.0</odometryRate>
  <publishOdometryTf>true</publishOdometryTf>
  <!-- the frame wich will be the odometry from, usually base_footprint -->
  <robotBaseFrame>link1</robotBaseFrame>
  <!-- Gaussian noise to apply on movement (can be interpreted as % of error) -->
  <gaussianNoiseXY>0.01</gaussianNoiseXY>
  <gaussianNoiseYaw>0.02</gaussianNoiseYaw>
  <!-- Limits to apply to the Twist commands -->
  <linearVelocityLimit>1.0</linearVelocityLimit>
  <angularVelocityLimit>3.0</angularVelocityLimit>
  <linearAccelerationLimit>1.0</linearAccelerationLimit>
  <angularAccelerationLimit>3.0</angularAccelerationLimit>
  <linearJerkLimit>5.0</linearJerkLimit>
  <angularJerkLimit>50.0</angularJerkLimit>
  </plugin>
</gazebo>
```