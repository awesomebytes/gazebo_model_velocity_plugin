/*
 * Copyright 2018 Sammy Pfeiffer, The Magic Lab, University of Technology Sydney
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to set
 *       velocities on a robot, resulting in motion. Also publishes
 *       odometry. Based on the
 *       gazebo_ros_force_based_move and ros_control speed_limit
 *       to implement velocity, acceleration and jerk limits.
 * Author: Sammy Pfeiffer
 * Date: 24 December 2018
 */

#include <math.h>
#include <gazebo_model_velocity_plugin/gazebo_ros_model_velocity.h>

namespace gazebo 
{

  GazeboRosModelVelocity::GazeboRosModelVelocity() {}

  GazeboRosModelVelocity::~GazeboRosModelVelocity() {}

  // Load the controller
  void GazeboRosModelVelocity::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {
    parent_ = parent;

    /* Parse parameters */
    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("GazeboRosModelVelocity missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") listening to Twist messages at: " << command_topic_);
    }

    output_vel_topic_ = "output_vel";
    if (!sdf->HasElement("outputVelocityTopic")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <outputVelocityTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), output_vel_topic_.c_str());
    } 
    else 
    {
      output_vel_topic_ = sdf->GetElement("outputVelocityTopic")->Get<std::string>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") publishing output Twist messages to: " << output_vel_topic_);
    }

    update_rate_ = 20.0;
    if (!sdf->HasElement("updateRate")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <updateRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), update_rate_);
    } 
    else 
    {
      update_rate_ = sdf->GetElement("updateRate")->Get<double>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") update rate is: " << update_rate_);
    } 

    command_timeout_ = 0.5;
    if (!sdf->HasElement("commandTimeout")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <commandTimeout>, "
          "defaults to %f",
          robot_namespace_.c_str(), command_timeout_);
    } 
    else 
    {
      command_timeout_ = sdf->GetElement("commandTimeout")->Get<double>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") command timeout is: " << command_timeout_);
    } 

    limiter_lin_ = gazebo_model_velocity_plugin::SpeedLimiter();
    limiter_ang_ = gazebo_model_velocity_plugin::SpeedLimiter();

    // Give some reasonable default
    limiter_lin_.max_velocity = 100.0;
    if (!sdf->HasElement("linearVelocityLimit")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <linearVelocityLimit>, "
          "defaults to %f",
          robot_namespace_.c_str(), limiter_lin_.max_velocity);
    } 
    else 
    {
      limiter_lin_.max_velocity = sdf->GetElement("linearVelocityLimit")->Get<double>();
      limiter_lin_.min_velocity = - limiter_lin_.max_velocity;
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <linearVelocityLimit> " << limiter_lin_.max_velocity);
    }

    // Give some reasonable default
    limiter_ang_.max_velocity = 10.0;
    if (!sdf->HasElement("angularVelocityLimit")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <angularVelocityLimit>, "
          "defaults to %f",
          robot_namespace_.c_str(), limiter_ang_.max_velocity);
    } 
    else 
    {
      limiter_ang_.max_velocity = sdf->GetElement("angularVelocityLimit")->Get<double>();
      limiter_ang_.min_velocity = - limiter_ang_.max_velocity;
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <angularVelocityLimit> " << limiter_ang_.max_velocity);

    } 

    // Give some reasonable default
    limiter_lin_.max_acceleration = 10.0;
    if (!sdf->HasElement("linearAccelerationLimit")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <linearAccelerationLimit>, "
          "defaults to %f",
          robot_namespace_.c_str(), limiter_lin_.max_acceleration);
    } 
    else 
    {
      limiter_lin_.max_acceleration = sdf->GetElement("linearAccelerationLimit")->Get<double>();
      limiter_lin_.min_acceleration = - limiter_lin_.max_acceleration;
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <linearAccelerationLimit> " << limiter_lin_.max_acceleration);
    }

    // Give some reasonable default
    limiter_ang_.max_acceleration = 10.0;
    if (!sdf->HasElement("angularAccelerationLimit")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <angularAccelerationLimit>, "
          "defaults to %f",
          robot_namespace_.c_str(), limiter_ang_.max_acceleration);
    } 
    else 
    {
      limiter_ang_.max_acceleration = sdf->GetElement("angularAccelerationLimit")->Get<double>();
      limiter_ang_.min_acceleration = - limiter_ang_.max_acceleration;
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <angularAccelerationLimit> " << limiter_ang_.max_acceleration);
    }


    // Give some reasonable default
    limiter_lin_.max_jerk = 100.0;
    if (!sdf->HasElement("linearJerkLimit")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <linearJerkLimit>, "
          "defaults to %f",
          robot_namespace_.c_str(), limiter_lin_.max_jerk);
    } 
    else 
    {
      limiter_lin_.max_jerk = sdf->GetElement("linearJerkLimit")->Get<double>();
      limiter_lin_.min_jerk = - limiter_lin_.max_jerk;
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <linearJerkLimit> " << limiter_lin_.max_jerk);

    }

    // Give some reasonable default
    limiter_ang_.max_jerk = 1000.0;
    if (!sdf->HasElement("angularJerkLimit")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <angularJerkLimit>, "
          "defaults to %f",
          robot_namespace_.c_str(), limiter_ang_.max_jerk);
    } 
    else 
    {
      limiter_ang_.max_jerk = sdf->GetElement("angularJerkLimit")->Get<double>();
      limiter_ang_.min_jerk = - limiter_ang_.max_jerk;
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <angularJerkLimit> " << limiter_ang_.max_jerk);

    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), odometry_topic_.c_str());
    } 
    else 
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <odometryTopic> " << odometry_topic_);

    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else 
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <odometryFrame> " << odometry_frame_);

    }

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <odometryRate> " << odometry_rate_);
    } 

    publish_odometry_tf_ = true;
    if (!sdf->HasElement("publishOdometryTf")) {
      ROS_WARN("GazeboRosModelVelocity Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
               robot_namespace_.c_str(), publish_odometry_tf_ ? "true" : "false");
    } else {
      publish_odometry_tf_ = sdf->GetElement("publishOdometryTf")->Get<bool>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <publishOdometryTf> " << publish_odometry_tf_);
    }

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame")) 
    {
      ROS_WARN("GazeboRosModelVelocity (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    } 
    else 
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
      ROS_INFO_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_ << ") has <robotBaseFrame> " << robot_base_frame_);
    }

    if (!sdf->HasElement("gaussianNoiseXY"))
    {
      ROS_WARN("GazeboRosModelVelocity missing <gaussianNoiseXY>, defaults to 0.0");
      gaussian_noise_xy_ = 0;
    }
    else{
      gaussian_noise_xy_ = sdf->GetElement("gaussianNoiseXY")->Get<double>();
      ROS_INFO_STREAM("GazeboRosModelVelocity has <gaussianNoiseXY> " << gaussian_noise_xy_);
    }

    if (!sdf->HasElement("gaussianNoiseYaw"))
    {
      ROS_WARN("GazeboRosModelVelocity missing <gaussianNoiseYaw>, defaults to 0.0");
      gaussian_noise_yaw_ = 0;
    }
    else{
      gaussian_noise_yaw_ = sdf->GetElement("gaussianNoiseYaw")->Get<double>();
      ROS_INFO_STREAM("GazeboRosModelVelocity has <gaussianNoiseYaw> " << gaussian_noise_yaw_);
    }

    seed = 0;

    // Enable the limits to be applied
    limiter_lin_.has_velocity_limits = true;
    limiter_lin_.has_acceleration_limits = true;
    limiter_lin_.has_jerk_limits = true;
    limiter_ang_.has_velocity_limits = true;
    limiter_ang_.has_acceleration_limits = true;
    limiter_ang_.has_jerk_limits = true;

    current_cmd_ = geometry_msgs::Twist();
    last_cmd0_ = geometry_msgs::Twist();
    last_cmd1_ = geometry_msgs::Twist();

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("GazeboRosModelVelocity (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    nh_.reset(new ros::NodeHandle(robot_namespace_));

    last_command_time_ = ros::Time::now();
    last_velocity_update_time_ = ros::Time::now();


    ROS_INFO("GazeboRosModelVelocity Plugin (%s) has started!", 
        robot_namespace_.c_str());

    // Subscribe to the cmd_vel topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosModelVelocity::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = nh_->subscribe(so);

    output_vel_pub_ = nh_->advertise<geometry_msgs::Twist>(output_vel_topic_, 1);

    odom_transform_.setIdentity();
    last_odom_publish_time_ = ros::Time::now();
    odometry_pub_ = nh_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    if (publish_odometry_tf_)
      transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // Start custom queue
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosModelVelocity::QueueThread, this));

    // Listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosModelVelocity::UpdateChild, this));

  }


  // Update the controller
  void GazeboRosModelVelocity::UpdateChild()
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    ros::Time tnow = ros::Time::now();
    ros::Duration dt = tnow - last_velocity_update_time_;
    geometry_msgs::Twist cmd = current_cmd_;

    if (dt > ros::Duration(1.0 / update_rate_)){
      // If we stop receiving commands, stop the robot
      if ((tnow - last_command_time_) > ros::Duration(command_timeout_)){
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
      }

      // Apply limits
      double x_lim_factor = limiter_lin_.limit(cmd.linear.x, last_cmd0_.linear.x, last_cmd1_.linear.x, dt.toSec());
      double y_lim_factor = limiter_lin_.limit(cmd.linear.y, last_cmd0_.linear.y, last_cmd1_.linear.y, dt.toSec());
      double theta_lim_factor = limiter_ang_.limit(cmd.angular.z, last_cmd0_.angular.z, last_cmd1_.angular.z, dt.toSec());

      last_cmd1_ = last_cmd0_;
      last_cmd0_ = cmd;

      // Apply velocity to the Model
      if (x_lim_factor != 1.0 || y_lim_factor != 1.0 || theta_lim_factor != 1.0)
        ROS_DEBUG_STREAM("Limiting factors: " <<
          "\nx:     " << x_lim_factor <<
          "\ny:     " << y_lim_factor <<
          "\ntheta: " << theta_lim_factor);

      // We can only set a linear velocity in reference of the world
      // with the Gazebo API, so we compute what is the linear velocity
      // given the angle of the robot
#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Pose3d pose = parent_->RelativePose();
      double yaw = pose.Rot().Yaw();
#else
      math::Pose pose = parent_->GetRelativePose();
      double yaw = pose.rot.GetYaw();
#endif
      double x_vel_cmd = cmd.linear.x * cos(yaw) + cmd.linear.y * sin(yaw);
      double y_vel_cmd = cmd.linear.x * sin(yaw) - cmd.linear.y * cos(yaw);

      parent_->SetLinearVel(math::Vector3(x_vel_cmd, y_vel_cmd, 0.0));
      parent_->SetAngularVel(math::Vector3(0.0, 0.0, cmd.angular.z));

      output_vel_pub_.publish(cmd);
      last_velocity_update_time_ = tnow;
    }

    if (odometry_rate_ > 0.0) {
      double seconds_since_last_update = 
        (tnow - last_odom_publish_time_).toSec();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = tnow;
      }

    }
  }

  // Finalize the controller
  void GazeboRosModelVelocity::FiniChild() {
    queue_.clear();
    queue_.disable();
    nh_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosModelVelocity::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    current_cmd_ = *cmd_msg;

    ROS_DEBUG_STREAM("Updating command:\n" << 
      "x: " << cmd_msg->linear.x <<
      "\ny: " << cmd_msg->linear.y <<
      "\nyaw: " << cmd_msg->angular.z);
    last_command_time_ = ros::Time::now();
 
  }

  // Utility for adding noise
  double GazeboRosModelVelocity::GaussianKernel(double mu, double sigma)
  {
    // using Box-Muller transform to generate two independent standard
    // normally disbributed normal variables see wikipedia

    // normalized uniform random variable
    double U = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    // normalized uniform random variable
    double V = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
    // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

    // there are 2 indep. vars, we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
  }

  void GazeboRosModelVelocity::publishOdometry(double step_time)
  {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = odometry_frame_;
    std::string base_footprint_frame = robot_base_frame_;

#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d angular_vel = parent_->RelativeAngularVel();
    ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

    odom_.twist.twist.angular.z = angular_vel.Z() + GaussianKernel(0, gaussian_noise_yaw_);
    odom_.twist.twist.linear.x  = linear_vel.X() + GaussianKernel(0, gaussian_noise_xy_);
    odom_.twist.twist.linear.y  = linear_vel.Y() + GaussianKernel(0, gaussian_noise_xy_);

#else
    math::Vector3 angular_vel = parent_->GetRelativeAngularVel();
    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    odom_.twist.twist.angular.z = angular_vel.z + GaussianKernel(0, gaussian_noise_yaw_);
    odom_.twist.twist.linear.x  = linear_vel.x + GaussianKernel(0, gaussian_noise_xy_);
    odom_.twist.twist.linear.y  = linear_vel.y + GaussianKernel(0, gaussian_noise_xy_);

#endif
    // If we are actually not moving, dont add noise, that makes no sense
    if (last_cmd0_.linear.x == 0.0)
      odom_.twist.twist.linear.x = 0.0;
    if (last_cmd0_.linear.y == 0.0)
      odom_.twist.twist.linear.y = 0.0;
    if (last_cmd0_.angular.z == 0.0)
      odom_.twist.twist.angular.z = 0.0;
    odom_transform_ = odom_transform_ * getTransformForMotion(odom_.twist.twist.linear.x, 
                                                              odom_.twist.twist.linear.y, 
                                                              odom_.twist.twist.angular.z,
                                                               step_time);
    tf::poseTFToMsg(odom_transform_, odom_.pose.pose);

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    if (transform_broadcaster_.get()){
      transform_broadcaster_->sendTransform(
          tf::StampedTransform(odom_transform_, current_time, odom_frame,
              base_footprint_frame));
    }

// TODO: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_p3d.cpp#L335-L348
// Fix the covariances?

    odom_.pose.covariance[0] = 0.001;
    odom_.pose.covariance[7] = 0.001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    
#if (GAZEBO_MAJOR_VERSION >= 8)
    if (std::abs(angular_vel.Z()) < 0.0001) {
#else
    if (std::abs(angular_vel.z) < 0.0001) {
#endif
      odom_.pose.covariance[35] = 0.01;
    }else{
      odom_.pose.covariance[35] = 100.0;
    }

    odom_.twist.covariance[0] = 0.001;
    odom_.twist.covariance[7] = 0.001;
    odom_.twist.covariance[14] = 0.001;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;

#if (GAZEBO_MAJOR_VERSION >= 8)
    if (std::abs(angular_vel.Z()) < 0.0001) {
#else
    if (std::abs(angular_vel.z) < 0.0001) {
#endif
      odom_.twist.covariance[35] = 0.01;
    }else{
      odom_.twist.covariance[35] = 100.0;
    }

    odometry_pub_.publish(odom_);
  }


  tf::Transform GazeboRosModelVelocity::getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const
  {
    tf::Transform tmp;
    tmp.setIdentity();


    if (std::abs(angular_vel) < 0.0001) {
      //Drive straight
      tmp.setOrigin(tf::Vector3(static_cast<double>(linear_vel_x*timeSeconds), static_cast<double>(linear_vel_y*timeSeconds), 0.0));
    } else {
      //Follow circular arc
      double distChange = linear_vel_x * timeSeconds + linear_vel_y * timeSeconds;
      double angleChange = angular_vel * timeSeconds;

      double arcRadius = distChange / angleChange;

      tmp.setOrigin(tf::Vector3(std::sin(angleChange) * arcRadius,
                                arcRadius - std::cos(angleChange) * arcRadius,
                                0.0));
      tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
    }

    return tmp;
  }


  void GazeboRosModelVelocity::QueueThread()
  {
    static const double timeout = 0.01;
    while (nh_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }


  GZ_REGISTER_MODEL_PLUGIN(GazeboRosModelVelocity)
}

