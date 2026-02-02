#include <franka_example_controllers/exercise_cartesian_impedance_controller.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>


inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, double lambda = 0.2) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda * lambda);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
ExerciseCartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ExerciseCartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type ExerciseCartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  // Get robot state
  // - compute the transformation base-to-EE from the robot model (FK)
  Eigen::Map<const Matrix4d> current_transform(
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  // - extract the current EE position from the transformation
  current_position_ = current_transform.block<3, 1>(0, 3);
  // - extract the current EE orientation from the transformation
  current_orientation_ = current_transform.block<3, 3>(0, 0);
  // - compute the EE Jacobian
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());
  Vector7d q;
  Vector7d dq;
  updateJointStates(q, dq);

  // Allocate torque vectors
  Vector7d tau_task, tau_nullspace, tau_d;
  tau_task.setZero();
  tau_nullspace.setZero();
  tau_d.setZero();
  Vector6d error;
  error.setZero();

  /* TODO: Compute the task torques  ----------------------------------------------------------------- */
  
  // Get the position error
  error.head(3) << position_d_ - current_position_;

  // Get the orientation error
  // - compute the difference between current and desired quaternions
  Eigen::Quaterniond error_quaternion = current_orientation_.inverse() * orientation_d_;
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // - transform it in robot base frame
  error.tail(3) << current_transform.block<3, 3>(0, 0) * error.tail(3);

  // Compute desired torques
  // - task control torques J^T * (K * err - D * vel)
  tau_task << jacobian.transpose() * (task_stiff_ * error - task_damp_ * (jacobian * dq));

  /* TODO: Compute the null-space torques  ------------------------------------------------------------ */

  // // Compute the Jacobian pseudoinverse
  // Eigen::MatrixXd jacobian_pinv;
  // pseudoInverse(jacobian, jacobian_pinv, 0.2);
  // // Null-space control torques
  // tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian) *
  //                  (ns_stiff_ * (q_d_nullspace_ - q) - ns_damp_ * dq);
  
  /* TODO: Filter the reference pose  ----------------------------------------------------------------- */

  position_d_ = filter_param_ * position_d_target_ + (1.0 - filter_param_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_param_, orientation_d_target_);

  /* ------------------------------------------------------------------------------------------------ */

  tau_d << tau_task + tau_nullspace;

  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn ExerciseCartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("topic", "/cartesian_impedance/desired_pose");
    
    sub_eq_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        get_node()->get_parameter("topic").as_string(), 1,
        std::bind(&ExerciseCartesianImpedanceController::equilibriumPoseCallback, this,
                  std::placeholders::_1));

    filt_ref_pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/cartesian_impedance/filt_ref_pose", 1);
    current_pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/cartesian_impedance/current_pose", 1);
  
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExerciseCartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    robot_type_ = get_node()->get_parameter("robot_type").as_string();

    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        franka_semantic_components::FrankaRobotModel(
            robot_type_ + "/" + k_robot_model_interface_name,
            robot_type_ + "/" + k_robot_state_interface_name));
    model_state_interface_count_ = franka_robot_model_->get_state_interface_names().size();
  
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during configuration stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExerciseCartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    // Set the starting EE pose as the initial reference for the controller
    init_pose_matrix_ =
        Matrix4d(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
    position_d_ = Vector3d(init_pose_matrix_.block<3, 1>(0, 3));
    position_d_target_ = position_d_;
    orientation_d_ = Quaterniond(init_pose_matrix_.block<3, 3>(0, 0));
    orientation_d_target_ = orientation_d_;

    // Set the starting joint configuration as the joint reference in the null-space
    Vector7d q;
    Vector7d dq;
    updateJointStates(q, dq);
    q_d_nullspace_ = q;

    // Build the stiffness and damping matrices for the controller
    task_stiff_.setIdentity();
    task_stiff_.topLeftCorner(3, 3) = Eigen::Map<Vector3d>(pos_stiff_.data()).asDiagonal();
    task_stiff_.bottomRightCorner(3, 3) = Eigen::Map<Vector3d>(rot_stiff_.data()).asDiagonal();
    task_damp_ = 2 * task_stiff_.array().sqrt();

    ns_stiff_ = Eigen::Map<Vector7d>(ns_joint_stiff_.data()).asDiagonal();
    ns_damp_ = 2 * ns_stiff_.array().sqrt();

    auto node = get_node();
    auto period = std::chrono::duration<double>(1.0 / pub_frequency_);

    pub_timer_ =
        node->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                std::bind(&ExerciseCartesianImpedanceController::publishData, this));

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during activation stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExerciseCartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

/* TODO: Setup the desired pose topic subscriber -------------------------------------------------- */

void ExerciseCartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::msg::PoseStamped& msg) {
  position_d_target_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  // coeffs() order is x, y, z, w
  orientation_d_target_.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w;
  orientation_d_target_.normalize();
}

/* ------------------------------------------------------------------------------------------------ */


void ExerciseCartesianImpedanceController::publishData() {
  // publish data
  geometry_msgs::msg::PoseStamped msgCartPosDesFilt;
  geometry_msgs::msg::PoseStamped msgCartPosCurr;

  auto stamp = get_node()->now();
  msgCartPosDesFilt.header.stamp = stamp;
  msgCartPosCurr.header.stamp = stamp;
  // Cartesian target pose filtered
  msgCartPosDesFilt.pose.position.x = position_d_(0);
  msgCartPosDesFilt.pose.position.y = position_d_(1);
  msgCartPosDesFilt.pose.position.z = position_d_(2);
  msgCartPosDesFilt.pose.orientation.x = orientation_d_.x();
  msgCartPosDesFilt.pose.orientation.y = orientation_d_.y();
  msgCartPosDesFilt.pose.orientation.z = orientation_d_.z();
  msgCartPosDesFilt.pose.orientation.w = orientation_d_.w();

  // Cartesian current pose
  msgCartPosCurr.pose.position.x = current_position_(0);
  msgCartPosCurr.pose.position.y = current_position_(1);
  msgCartPosCurr.pose.position.z = current_position_(2);
  msgCartPosCurr.pose.orientation.x = current_orientation_.x();
  msgCartPosCurr.pose.orientation.y = current_orientation_.y();
  msgCartPosCurr.pose.orientation.z = current_orientation_.z();
  msgCartPosCurr.pose.orientation.w = current_orientation_.w();

  filt_ref_pose_pub_->publish(msgCartPosDesFilt);
  current_pose_pub_->publish(msgCartPosCurr);
}

void ExerciseCartesianImpedanceController::updateJointStates(Vector7d& q, Vector7d& dq) const {
  for (int i = 0; i < num_joints_; ++i) {
    const auto& position_interface = state_interfaces_.at(model_state_interface_count_ + 2 * i);
    const auto& velocity_interface =
        state_interfaces_.at(model_state_interface_count_ + 2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q(i) = position_interface.get_value();
    dq(i) = velocity_interface.get_value();
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ExerciseCartesianImpedanceController,
                       controller_interface::ControllerInterface)
