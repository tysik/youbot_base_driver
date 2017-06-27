/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "youbot_base_driver/youbot_base_driver.h"

using namespace youbot_base_driver;
using namespace std;

YoubotBaseDriver::YoubotBaseDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  youbot::Logger::toConsole = false;
  youbot::Logger::toFile = false;
  youbot::Logger::toROS = true;

  nh_local_.param<bool>("publish_tf", p_publish_tf_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<string>("parent_frame_id", p_parent_frame_id_, "odom");
  nh_local_.param<string>("child_frame_id", p_child_frame_id_, "base");

  nh_local_.param<string>("config_file_path", p_config_file_path_, "config");
  nh_local_.param<string>("config_file_name", p_config_file_name_, "youbot-base");

  try {
    youbot_base_ = new youbot::YouBotBase(p_config_file_name_, p_config_file_path_);
    youbot_base_->doJointCommutation();
  }
  catch (std::exception& e) {
    throw e.what();
  }

  timer_ = nh_.createTimer(ros::Duration(1.0), &YoubotBaseDriver::timerCallback, this, false, false);
  controls_sub_ = nh_.subscribe("controls", 10, &YoubotBaseDriver::controlsCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);
  timer_.start();
}

YoubotBaseDriver::~YoubotBaseDriver() {
  if (youbot_base_) {
    delete youbot_base_;
    youbot_base_ = 0;
  }

  timer_.stop();
  controls_sub_.shutdown();
  odom_pub_.shutdown();

  youbot::EthercatMaster::destroy();
}

void YoubotBaseDriver::controlsCallback(const geometry_msgs::Twist::ConstPtr controls_msg) {
  quantity<si::velocity> u;
  quantity<si::velocity> v;
  quantity<si::angular_velocity> w;

  u = controls_msg->linear.x * meter_per_second;
  v = controls_msg->linear.y * meter_per_second;
  w = controls_msg->angular.z * radian_per_second;

  try {
    youbot_base_->setBaseVelocity(u, v, w);
  }
  catch (std::exception& e) {
    ROS_ERROR_STREAM(e.what());
  }
}

void YoubotBaseDriver::timerCallback(const ros::TimerEvent& e) {
  try {
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);

    quantity<si::length> longitudinalPosition;
    quantity<si::length> transversalPosition;
    quantity<plane_angle> orientation;

    quantity<si::velocity> longitudinalVelocity;
    quantity<si::velocity> transversalVelocity;
    quantity<si::angular_velocity> angularVelocity;

    youbot_base_->getBasePosition(longitudinalPosition, transversalPosition, orientation);
    double x = longitudinalPosition.value();
    double y = transversalPosition.value();
    double theta = orientation.value();

    youbot_base_->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
    double u = longitudinalVelocity.value();
    double v = transversalVelocity.value();
    double w = angularVelocity.value();

    ros::Time now = ros::Time::now();
    geometry_msgs::Quaternion rotation = tf::createQuaternionMsgFromYaw(theta);

    if (p_publish_tf_) {
      geometry_msgs::TransformStamped odom_tf;

      odom_tf.header.stamp = now;
      odom_tf.header.frame_id = p_parent_frame_id_;
      odom_tf.child_frame_id = p_child_frame_id_;

      odom_tf.transform.translation.x = x;
      odom_tf.transform.translation.y = y;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = rotation;

      tf_bc_.sendTransform(odom_tf);
    }

    nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry);

    odom_msg->header.stamp = now;
    odom_msg->header.frame_id = p_parent_frame_id_;
    odom_msg->child_frame_id = p_child_frame_id_;

    odom_msg->pose.pose.position.x = x;
    odom_msg->pose.pose.position.y = y;
    odom_msg->pose.pose.position.z = 0.0;
    odom_msg->pose.pose.orientation = rotation;

    odom_msg->twist.twist.linear.x = u;
    odom_msg->twist.twist.linear.y = v;
    odom_msg->twist.twist.angular.z = w;

    odom_pub_.publish(odom_msg);

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);
  }
  catch (youbot::EtherCATConnectionException& e) {
    ROS_ERROR_STREAM(e.what());
  }
  catch (std::exception& e)	{
    ROS_ERROR_STREAM(e.what());
  }
}
