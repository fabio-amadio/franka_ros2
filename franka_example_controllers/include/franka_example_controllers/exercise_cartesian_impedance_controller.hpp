#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {
using Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

using Eigen::Quaterniond;


class ExerciseCartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  const int num_joints_ = 7;

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  Quaterniond orientation_d_;
  Quaterniond orientation_d_target_;
  Vector3d position_d_;
  Vector3d position_d_target_;
  Vector7d q_d_nullspace_;
  Matrix4d init_pose_matrix_;
  
  std::string robot_type_;
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  size_t model_state_interface_count_{0};
  Matrix6d task_stiff_;
  Matrix6d task_damp_;
  Matrix7d ns_stiff_;
  Matrix7d ns_damp_;
  
  // translation stiffness (cartesian task)
  std::vector<double> pos_stiff_{800.0, 800.0, 800.0};  // axes x, y, z
  // roation stiffness (cartesian task)
  std::vector<double> rot_stiff_{60.0, 60.0, 60.0};  // axes x, y, z
  // joint stiffness (null-space)
  std::vector<double> ns_joint_stiff_{5.0, 5.0, 5.0, 5.0, 0.1, 0.1, 0.1}; // joints q1, ... q7
  double filter_param_{0.008};

  // Desired pose subscriber
  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped& msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_eq_pose_;

  // Current pose publisher
  void publishData(void);
  double pub_frequency_{30.0};
  rclcpp::TimerBase::SharedPtr pub_timer_;
  Vector3d current_position_;
  Quaterniond current_orientation_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filt_ref_pose_pub_;

  void updateJointStates(Vector7d& q, Vector7d& dq) const;
};

}  // namespace franka_example_controllers
