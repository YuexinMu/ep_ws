//
// Created by myx on 2024/5/27.
//

#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>


namespace ep_gazebo
{
class EptOdomSim:public gazebo::ModelPlugin{
public:
  EptOdomSim() {}
  ~EptOdomSim() {
    node_handle_->shutdown();
    delete node_handle_;
  }

  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

  virtual void Update();

private:
  void QueueThread();

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr robot_link_;
  gazebo::event::ConnectionPtr update_connection_;
  ros::NodeHandle* node_handle_;

  nav_msgs::Odometry odom_msg_;
  ros::Publisher odom_pub_;
  geometry_msgs::TransformStamped odom2base_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Time last_publish_time_;

  // parameters
  double publish_rate_;
  std::string robot_namespace_;
  tf2::Quaternion rpy_to_quat_;
};
}

