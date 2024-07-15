//
// Created by myx on 2023/10/8.
//

#pragma once

#include <Eigen/Dense>
#include <gazebo_ros_control/default_robot_hw_sim.h>
//#include <sct_common/hardware_interface/scout_interface.h>

#include <effort_controllers/joint_effort_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <geometry_msgs/Twist.h>

namespace ep_gazebo
{
// TODO(MuYuexin) Don't know why the EpRobotHWSim initialisation process the JointVelocityController doesn't work
// so there write a SimpleJointVelocityController
class SimpleJointVelocityController
{
public:
  SimpleJointVelocityController();
  ~SimpleJointVelocityController();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n);

  /*!
   * \brief Give set velocity of the joint for next update: revolute (angle) and prismatic (velocity)
   *
   * \param double pos Velocity command to issue
   */
  void setCommand(double cmd);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Get the name of the joint this controller uses
   */
  std::string getJointName();

  double getVelocity();

  hardware_interface::JointHandle joint_;
  double command_; /**< Last commanded velocity. */

private:
  control_toolbox::Pid pid_controller_; /**< Internal PID controller. */
};

class EpRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;
  void readSim(ros::Time time, ros::Duration period) override;

  void writeSim(ros::Time time, ros::Duration period) override;

private:
//  sct_common::ScoutCommandData sct_command_data_;
//  sct_common::ScoutInterface scout_interface_;

  control_toolbox::Pid angle_pid_controller_;
  gazebo::physics::LinkPtr base_link_;
  Eigen::MatrixXd chassis2joints_;

  std::vector<std::shared_ptr<SimpleJointVelocityController>> joints_;
  std::vector<double> joint2trans_;

  double wheel_base_{}, wheel_track_{}, wheel_radius_{}, publish_rate_{}, twist_angular_{}, timeout_{}, effort_coeff_{},
      velocity_coeff_{}, power_offset_{};

  ros::Subscriber cmd_vel_sub_;
  geometry_msgs::Twist cmd_vel_;
  bool update_cmd_;

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

}  // namespace ep_gazebo
