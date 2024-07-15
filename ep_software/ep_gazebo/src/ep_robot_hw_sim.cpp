//
// Created by myx on 2023/10/8.
//

#include "ep_gazebo/ep_robot_hw_sim.h"

#include <string>
#include <tinyxml.h>
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <control_toolbox/pid.h>

namespace ep_gazebo
{
bool EpRobotHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                            gazebo::physics::ModelPtr parent_model, const urdf::Model* urdf_model,
                            std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);

  if (!model_nh.hasParam("angular_control"))
    ROS_WARN_STREAM("No " << model_nh.getNamespace() <<" angular_control specified");
  if (!angle_pid_controller_.init(ros::NodeHandle(model_nh, "angular_control/pid")))
    return false;

  base_link_ = parent_model->GetLink("base_link");

  XmlRpc::XmlRpcValue wheels;

  ROS_INFO_STREAM("Get robot: " << model_nh.getNamespace() << " successfully");
  if (!model_nh.getParam("wheels", wheels))
    ROS_WARN("No wheels specified");
  else
  {
    chassis2joints_.resize(wheels.size(), 3);
    size_t i = 0;
    for (const auto& wheel : wheels)
    {
      ROS_ASSERT(wheel.second.hasMember("pose"));
      ROS_ASSERT(wheel.second["pose"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(wheel.second["pose"].size() == 3);
      ROS_ASSERT(wheel.second.hasMember("roller_angle"));
      ROS_ASSERT(wheel.second.hasMember("radius"));

      // Ref: Modern Robotics, Chapter 13.2: Omnidirectional Wheeled Mobile Robots
      Eigen::MatrixXd direction(1, 2), in_wheel(2, 2), in_chassis(2, 3);
      double beta = (double)wheel.second["pose"][2];
      double roller_angle = (double)wheel.second["roller_angle"];
      direction << 1, tan(roller_angle);
      in_wheel << cos(beta), sin(beta), -sin(beta), cos(beta);
      in_chassis << -(double)wheel.second["pose"][1], 1., 0., (double)wheel.second["pose"][0], 0., 1.;
      Eigen::MatrixXd chassis2joint = 1. / (double)wheel.second["radius"] * direction * in_wheel * in_chassis;
      chassis2joints_.block<1, 3>(i, 0) = chassis2joint;

      ros::NodeHandle nh_wheel = ros::NodeHandle(model_nh, "wheels/" + wheel.first);
      joints_.push_back(std::make_shared<SimpleJointVelocityController>());
      if (!joints_.back()->init(&ej_interface_, nh_wheel))
        return false;

      i++;
    }
  }

//  sct_command_data_.stamp = ros::Time::now();
//  sct_command_data_.motion_ctl_cmd.linear_vel = 0;
//  sct_command_data_.motion_ctl_cmd.angle_vel = 0;
//  sct_command_data_.motion_ctl_cmd.update_cmd = false;
//  sct_command_data_.ctl_mode_set_cmd.control_mode = 0;
//  sct_command_data_.ctl_mode_set_cmd.update_cmd = false;
//  sct_command_data_.state_set_cmd.state_set = 0;
//  sct_command_data_.state_set_cmd.update_cmd = false;
//  sct_command_data_.light_set_cmd.light_ctl_enable = 0;
//  sct_command_data_.light_set_cmd.light_mode = sct_common::OFF;
//  sct_command_data_.light_set_cmd.user_define_light = 0;
//  sct_command_data_.light_set_cmd.count_check = 0;
//  sct_command_data_.light_set_cmd.update_cmd = false;

//  sct_common::ScoutHandle scout_handle(model_nh.getNamespace(), &sct_command_data_);
//  scout_interface_.registerHandle(scout_handle);
//  registerInterface(&scout_interface_);

  cmd_vel_sub_ = model_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &EpRobotHWSim::cmdVelCallback, this);
  update_cmd_ = false;
  return ret;
}

void EpRobotHWSim::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!update_cmd_){
    cmd_vel_ = *msg;
    update_cmd_ = true;
  }
}

void EpRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto& cmd : joint_effort_command_)
    cmd = 0;
  for (auto& cmd : joint_velocity_command_)
    cmd = 0;
}

void EpRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  if (update_cmd_){
    geometry_msgs::Twist cmd_ = cmd_vel_;
    update_cmd_ = false;

    double error = cmd_.angular.z - base_link_->RelativeAngularVel().Z();
    cmd_.angular.z = angle_pid_controller_.computeCommand(error, period);

    Eigen::Vector3d vel_chassis;
    vel_chassis << cmd_.angular.z, cmd_.linear.x, cmd_.linear.y;
    Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;
    for (size_t i = 0; i < joints_.size(); i++)
    {
      joints_[i]->setCommand(vel_joints(i));
      joints_[i]->update(time, period);
    }

//  double error = sct_command_data_.motion_ctl_cmd.angle_vel - base_link_->RelativeAngularVel().Z();
//  sct_command_data_.motion_ctl_cmd.angle_vel = angle_pid_controller_.computeCommand(error, period);
//  if (sct_command_data_.motion_ctl_cmd.update_cmd)
//  {
//    Eigen::Matrix<double, 2, 1>  vel_chassis;
//    vel_chassis << sct_command_data_.motion_ctl_cmd.linear_vel, sct_command_data_.motion_ctl_cmd.angle_vel;
//    Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;
//
//    for (size_t i = 0; i < joints_.size(); i++)
//    {
//      joints_[i]->setCommand(vel_joints(i)*joint2trans_[i]);
//      joints_[i]->update(time, period);
//    }
//    sct_command_data_.motion_ctl_cmd.update_cmd = false;
//  }
  }
  gazebo_ros_control::DefaultRobotHWSim::writeSim(time, period);
}

SimpleJointVelocityController::SimpleJointVelocityController() : command_(0)
{
}

bool SimpleJointVelocityController::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  return true;
}

// Set the joint velocity command
void SimpleJointVelocityController::setCommand(double cmd)
{
  command_ = cmd;
}

void SimpleJointVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
  double error = command_ - joint_.getVelocity();

  // Set the PID error and compute the PID command with nonuniform time
  // step size. The derivative error is computed from the change in the error
  // and the timestep dt.
  double commanded_effort = pid_controller_.computeCommand(error, period);
  joint_.setCommand(commanded_effort);
}

std::string SimpleJointVelocityController::getJointName()
{
  return joint_.getName();
}

double SimpleJointVelocityController::getVelocity()
{
  return joint_.getVelocity();
}
}  // namespace ep_gazebo

PLUGINLIB_EXPORT_CLASS(ep_gazebo::EpRobotHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin
