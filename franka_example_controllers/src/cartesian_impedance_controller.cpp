#include <franka_example_controllers/cartesian_impedance_controller.hpp>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <string>
#include <tuple>
#include <type_traits>

#include <franka/robot_state.h>

namespace franka_example_controllers {
namespace {

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value &&
                     std::is_trivially_copyable<To>::value,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible<To>::value,
                "This implementation additionally requires "
                "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // namespace

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  const auto cartesian_pose_state_names = franka_cartesian_pose_->get_state_interface_names();
  config.names.insert(config.names.end(), cartesian_pose_state_names.begin(),
                      cartesian_pose_state_names.end());
  return config;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Get robot state
  // - get the current EE pose from the cartesian pose interface
  std::tie(current_orientation_, current_position_) =
      franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  current_orientation_.normalize();
  const Eigen::Matrix3d current_rotation = current_orientation_.toRotationMatrix();
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

  /* Compute the task torques  ----------------------------------------------------------------- */
  
  // Get the position error
  error.head(3) << position_d_ - current_position_;

  // Get the orientation error
  // - compute the difference between current and desired quaternions
  if (orientation_d_.coeffs().dot(current_orientation_.coeffs()) < 0.0) {
    orientation_d_.coeffs() << -orientation_d_.coeffs();
  }
  Eigen::Quaterniond error_quaternion = current_orientation_.inverse() * orientation_d_;
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // - transform it in robot base frame
  error.tail(3) << current_rotation * error.tail(3);
  const double rotation_error_norm = error.tail(3).norm();

  const Vector6d max_error = Eigen::Map<const Vector6d>(max_error_.data());
  for (int i = 0; i < 6; ++i) {
    error(i) = std::clamp(error(i), -max_error(i), max_error(i));
  }

  // Compute desired torques
  tau_task << jacobian.transpose() * (task_stiff_ * error - task_damp_ * (jacobian * dq));

  /* Compute the null-space torques  ------------------------------------------------------------ */

  const double lambda = 0.01;
  const Eigen::Matrix<double, 6, 6> jj_t =
      jacobian * jacobian.transpose() +
      (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 7> jj_t_inv_j = jj_t.ldlt().solve(jacobian);
  const Eigen::Matrix<double, 7, 7> nullspace_projector =
      Eigen::Matrix<double, 7, 7>::Identity() - jacobian.transpose() * jj_t_inv_j;
  // Null-space control torques
  tau_nullspace << nullspace_projector * (ns_stiff_ * (q_d_nullspace_ - q) - ns_damp_ * dq);
  for (int i = 0; i < num_joints_; ++i) {
    tau_nullspace(i) = std::clamp(tau_nullspace(i), -max_ns_torque_, max_ns_torque_);
  }
  
  /* Filter the reference pose  ----------------------------------------------------------------- */

  position_d_ = filter_param_ * position_d_target_ + (1.0 - filter_param_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_param_, orientation_d_target_);
  orientation_d_.normalize();

  /* ------------------------------------------------------------------------------------------------ */

  const auto coriolis_array = franka_robot_model_->getCoriolisForceVector();
  const Eigen::Map<const Vector7d> coriolis(coriolis_array.data());
  const Eigen::Map<const Vector7d> tau_J_d(robot_state_->tau_J_d.data());

  tau_d << tau_task + tau_nullspace + coriolis;
  tau_d = saturateTorqueRate(tau_d, tau_J_d);

  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("topic", "/cartesian_impedance/desired_pose");

    franka_cartesian_pose_ =
        std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
            franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));
    
    sub_eq_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        get_node()->get_parameter("topic").as_string(), 1,
        std::bind(&CartesianImpedanceController::equilibriumPoseCallback, this,
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

CallbackReturn CartesianImpedanceController::on_configure(
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

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
    updateRobotStatePointer();
    if (robot_state_ == nullptr) {
      RCLCPP_ERROR(get_node()->get_logger(), "Robot state interface not found.");
      return CallbackReturn::ERROR;
    }

    // Set the starting EE pose as the initial reference for the controller
    init_pose_matrix_ =
        Matrix4d(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
    position_d_ = Vector3d(init_pose_matrix_.block<3, 1>(0, 3));
    position_d_target_ = position_d_;
    orientation_d_ = Quaterniond(init_pose_matrix_.block<3, 3>(0, 0));
    orientation_d_target_ = orientation_d_;
    orientation_d_.normalize();
    orientation_d_target_.normalize();

    // Set the starting joint configuration as the joint reference in the null-space
    Vector7d q;
    Vector7d dq;
    updateJointStates(q, dq);
    q_d_nullspace_ = q;

    // Build the stiffness and damping matrices for the controller
    task_stiff_.setIdentity();
    task_stiff_.topLeftCorner(3, 3) = Eigen::Map<Vector3d>(pos_stiff_.data()).asDiagonal();
    task_stiff_.bottomRightCorner(3, 3) = Eigen::Map<Vector3d>(rot_stiff_.data()).asDiagonal();
    task_damp_ = 2.0 * task_stiff_.array().sqrt();

    ns_stiff_ = Eigen::Map<Vector7d>(ns_joint_stiff_.data()).asDiagonal();
    ns_damp_ = 2 * ns_stiff_.array().sqrt();

    auto node = get_node();
    auto period = std::chrono::duration<double>(1.0 / pub_frequency_);

    pub_timer_ =
        node->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                std::bind(&CartesianImpedanceController::publishData, this));

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during activation stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  franka_robot_model_->release_interfaces();
  robot_state_ = nullptr;
  return CallbackReturn::SUCCESS;
}

/* Setup the desired pose topic subscriber -------------------------------------------------- */

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::msg::PoseStamped& msg) {
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }
  position_d_target_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  // coeffs() order is x, y, z, w
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
  orientation_d_target_.normalize();
}

/* ------------------------------------------------------------------------------------------------ */


void CartesianImpedanceController::publishData() {
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }
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

void CartesianImpedanceController::updateJointStates(Vector7d& q, Vector7d& dq) const {
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

void CartesianImpedanceController::updateRobotStatePointer() {
  auto franka_state_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get_name() == robot_type_ + "/" + k_robot_state_interface_name;
      });

  if (franka_state_interface == state_interfaces_.end()) {
    robot_state_ = nullptr;
    return;
  }

  robot_state_ = bit_cast<franka::RobotState*>((*franka_state_interface).get_value());
}

Vector7d CartesianImpedanceController::saturateTorqueRate(
    const Vector7d& tau_d_calculated,
    const Vector7d& tau_J_d) const {
  Vector7d tau_d_saturated;
  for (int i = 0; i < num_joints_; ++i) {
    const double diff = tau_d_calculated(i) - tau_J_d(i);
    const double diff_limited = std::max(std::min(diff, delta_tau_max_), -delta_tau_max_);
    tau_d_saturated(i) = tau_J_d(i) + diff_limited;
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
