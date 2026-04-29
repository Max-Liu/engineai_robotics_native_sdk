/**
 * @file motion_stamp_runner.cc
 * @brief Implementation of the RL-based whole-body tracking (WBT) dance Runner.
 *
 * This Runner deploys a reinforcement-learning policy that tracks a pre-recorded
 * reference dance trajectory (loaded from a .npz file). Unlike the walking Runner
 * which uses live gamepad commands, this Runner replays a fixed motion sequence
 * so the robot can perform choreographed dance moves.
 *
 * Core pipeline each control cycle:
 *   1. Compute observations via a registry-based observation system (motion_stamp_obs)
 *   2. Run the MLP policy network inference to get joint actions
 *   3. Map actions to target joint positions and send motor commands
 *
 * Key differences from the walking Runner:
 *   - Uses a **reference trajectory** (.npz) instead of real-time gamepad input
 *   - Observation assembly is **registry-driven** — each observation component is
 *     registered by name and retrieved dynamically via motion_stamp_obs::GetObservation()
 *   - Supports **per-component history buffers** with configurable history lengths
 *   - Performs **yaw alignment** on the first frame to align the reference trajectory
 *     with the robot's actual heading at startup
 *
 * Notes for secondary developers:
 *   - To add new observation types, register them in motion_stamp_obs_registry and add the
 *     name to the `observation_names` parameter list in the YAML config.
 *   - The reference trajectory .npz file must contain keys: "joint_pos", "joint_vel",
 *     "body_quat_w" as float arrays.
 *   - Joint name ordering in `joint_names` must match the policy training configuration.
 */

#include "motion_stamp/motion_stamp_runner.h"

#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "math/interpolation.h"
#include "math/rotation_matrix.h"
#include "motion_stamp/motion_stamp_obs_registry.h"

namespace {

double WrapToPi(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

}  // namespace

namespace runner {

// ============================================================================
// Runner Lifecycle Methods
// ============================================================================

/**
 * @brief Sets up the runtime context before this Runner is scheduled.
 *
 * Disables the classic parser's parallel motion control, as the RL policy
 * has exclusive control over all active joints.
 */
void MotionStampRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(false); }

/**
 * @brief Tears down the runtime context. Currently no cleanup needed.
 */
void MotionStampRunner::TeardownContext() {}

/**
 * @brief Initialization upon entering this Runner. Allocates all resources.
 *
 * Performs the following steps:
 *   1. Load/reload parameters (supports param_tag_ hot-switching)
 *   2. Set up joint PD gains and build policy-to-deploy joint index mapping
 *   3. Load the MLP policy network (.mnn model)
 *   4. Initialize observation and history buffers
 *   5. Load the reference dance trajectory from a .npz file
 *   6. Pre-fill the constant portion of the observation context
 *
 * @return true if initialization succeeds, false on parameter or model load failure.
 */
bool MotionStampRunner::Enter() {
  // --- Step 1: Parameter loading ---
  // Reload parameters if a tag has been set (supports runtime config switching)
  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::MotionStampParam>(param_tag_);
  }
  if (!param_) {
    LOG(ERROR) << "[MotionStampRunner::Enter] Failed to create MotionStampParam";
    return false;
  }

  // --- Step 2: Joint PD gains and index mapping ---
  // Initialize full-body joint arrays (all joints, not just policy-controlled ones)
  joint_kp_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  joint_kd_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  default_joint_q_ = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Zero(model_param_->num_total_joints));

  // Build the mapping from policy action indices to full-body joint indices.
  // policy2deploy_joint_idx_[i] gives the full-body joint index for the i-th policy output.
  policy2deploy_joint_idx_ = std::make_shared<Eigen::VectorXi>(Eigen::VectorXi::Zero(param_->num_actions));
  for (size_t i = 0; i < param_->joint_names.size(); ++i) {
    int deploy_idx = model_param_->joint_id_in_total_limb.at(param_->joint_names[i]);
    (*policy2deploy_joint_idx_)(static_cast<int>(i)) = deploy_idx;
  }

  // Apply PD gains and default positions only to the policy-controlled joints
  joint_kp_(*policy2deploy_joint_idx_) = param_->joint_stiffness;
  joint_kd_(*policy2deploy_joint_idx_) = param_->joint_damping;
  (*default_joint_q_)(*policy2deploy_joint_idx_) = param_->default_joint_pos;
  action_scale_ = param_->action_scale;

  // --- Step 3: Load the MLP policy network ---
  std::string policy_path =
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->policy_file);
  mlp_net_ = std::make_unique<math::MNNModel>(policy_path);
  if (!mlp_net_) {
    LOG(ERROR) << "[WbtRunner::Enter] Failed to load policy model";
    return false;
  }

  // Compute the total observation dimension (sum of all observation components × their history lengths)
  int total_obs_dim = ComputeTotalObservationDim();
  mlp_net_observation_vec.setZero(total_obs_dim);
  mlp_net_action_ = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Zero(param_->num_actions));

  // --- Step 4: Initialize observation history buffers ---
  // Each observation component has its own sliding-window history buffer
  initHistoryBuffers();

  // --- Step 5: Load reference dance trajectory ---
  // Supported trajectory schemas:
  //   "joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w"
  //   "joint_pos", "joint_vel", "root_pos",   "root_rot",     "root_lin_vel"
  std::string traj_path =
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->trajectory_file_npz);
  try {
    trajectory_npz = cnpy::npz_load(traj_path);

    Eigen::MatrixXd ref_joint_pos = ReorderTrajectoryJointColumns(LoadTrajectoryMatrix({"joint_pos"}), "joint_pos");
    Eigen::MatrixXd ref_joint_vel = ReorderTrajectoryJointColumns(LoadTrajectoryMatrix({"joint_vel"}), "joint_vel");
    ref_joint_pos_all_ = std::make_shared<const Eigen::MatrixXd>(std::move(ref_joint_pos));
    ref_joint_vel_all_ = std::make_shared<const Eigen::MatrixXd>(std::move(ref_joint_vel));
    ref_body_pos_w_all_ = std::make_shared<const Eigen::MatrixXd>(LoadTrajectoryMatrix({"body_pos_w", "root_pos"}));
    ref_body_quat_w_all_ = std::make_shared<const Eigen::MatrixXd>(LoadTrajectoryMatrix({"body_quat_w", "root_rot"}));
    ref_body_lin_vel_w_all_ =
        std::make_shared<const Eigen::MatrixXd>(LoadTrajectoryMatrix({"body_lin_vel_w", "root_lin_vel"}));
  } catch (const std::exception& e) {
    LOG(ERROR) << "[WbtRunner::Enter] Failed to load trajectory: " << e.what();
    return false;
  }
  max_policy_step = ref_joint_pos_all_->rows() - 1;

  // --- Step 6: Reset runtime state ---
  is_first_time_ = true;
  policy_step = 0;
  log_first_policy_command_ = true;
  transition_iter_ = 0;
  transition_duration_s_ = std::max(0.0, param_->transition_duration_s.value_or(0.0));
  entry_transition_active_ = transition_duration_s_ > 0.0;

  data_store_->joint_info.GetState(data::JointInfoType::kPosition, transition_start_q_);
  transition_target_q_ = *default_joint_q_;
  if (param_->resident_control) {
    transition_target_q_(*policy2deploy_joint_idx_) = ref_joint_pos_all_->row(0);
  }

  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  GetMutableOutput().SetCommand(transition_start_q_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);

  // Pre-fill observation context fields that remain constant throughout execution
  fillObsContextConstantPart();

  LOG(INFO) << "[WbtRunner::Enter] Done, obs_dim=" << total_obs_dim << ", actions=" << param_->num_actions
            << ", frames=" << ref_joint_pos_all_->rows() << ", transition_duration_s=" << transition_duration_s_
            << ", transition_delta_norm=" << (transition_target_q_ - transition_start_q_).norm()
            << ", transition_delta_max_abs=" << (transition_target_q_ - transition_start_q_).cwiseAbs().maxCoeff();
  return true;
}

/**
 * @brief Pre-fills the observation context with references that do not change
 *        between control cycles.
 *
 * The ObsContext struct is shared with all observation computation functions
 * registered in the motion_stamp_obs system. This method sets the "constant" fields
 * (data store reference, trajectory data, joint mappings, etc.) so that only
 * the per-cycle fields (like policy_step) need updating in the main loop.
 */
void MotionStampRunner::fillObsContextConstantPart() {
  obs_ctx_.data_store = data_store_;
  obs_ctx_.ref_joint_pos_all = ref_joint_pos_all_;
  obs_ctx_.ref_joint_vel_all = ref_joint_vel_all_;
  obs_ctx_.ref_body_pos_w_all = ref_body_pos_w_all_;
  obs_ctx_.ref_body_quat_w_all = ref_body_quat_w_all_;
  obs_ctx_.ref_body_lin_vel_w_all = ref_body_lin_vel_w_all_;
  obs_ctx_.num_actions = param_->num_actions;
  obs_ctx_.default_joint_q = default_joint_q_;
  obs_ctx_.policy2deploy_joint_idx = policy2deploy_joint_idx_;
  obs_ctx_.actions = mlp_net_action_;
}

// ============================================================================
// Main Control Loop
// ============================================================================

/**
 * @brief Main loop called once per control cycle.
 *
 * Executes the perception → decision → action pipeline and advances the
 * trajectory frame counter. When the trajectory reaches its final frame,
 * the runner signals kTryExit so the FSM scheduler transitions to the next runner.
 */
void MotionStampRunner::Run() {
  if (RunEntryTransition()) {
    return;
  }

  CalculateObservation();   // Assemble observation from registered components
  CalculateMotorCommand();  // Run policy inference and compute target positions
  SendMotorCommand();       // Send PD commands to motors

  // Advance trajectory frame. Once the trajectory reaches the final frame,
  // trigger an automatic exit so the FSM scheduler transitions to the next runner.
  if (policy_step >= max_policy_step) {
    SetRunnerState(RunnerState::kTryExit);
  } else {
    policy_step++;
  }
}

bool MotionStampRunner::RunEntryTransition() {
  if (!entry_transition_active_) {
    return false;
  }

  const double phase = std::min(static_cast<double>(transition_iter_) * runner_period_, transition_duration_s_);
  math::QuinticInterpolate(transition_start_q_, transition_target_q_, transition_duration_s_, phase, q_des_, qd_des_);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);

  ++transition_iter_;
  if (phase >= transition_duration_s_) {
    entry_transition_active_ = false;
    is_first_time_ = true;
    policy_step = 0;
    mlp_net_action_->setZero();
    LOG(INFO) << "[WbtRunner::EntryTransition] Completed, duration_s=" << transition_duration_s_
              << ", iterations=" << transition_iter_;
  }
  return true;
}

// ============================================================================
// Observation Assembly
// ============================================================================

/**
 * @brief Assembles the full observation vector from registry-based observation components.
 *
 * Unlike the walking Runner which manually concatenates sensor data, this Runner
 * uses a dynamic registry system (motion_stamp_obs). Each observation component is:
 *   1. Looked up by name from the `observation_names` config list
 *   2. Computed by its registered function via motion_stamp_obs::GetObservation()
 *   3. Maintained in its own sliding-window history buffer
 *   4. Flattened (column-major) into the final observation vector
 *
 * On the first frame:
 *   - Yaw alignment is performed to match the reference trajectory heading
 *   - All history buffers are pre-filled with the current observation
 *     (avoids feeding zero-initialized history to the policy)
 *
 * @note The observation composition and ordering are fully determined by the YAML
 *       config parameter `observation_names`. Adding or reordering entries requires
 *       retraining the policy model.
 */
void MotionStampRunner::CalculateObservation() {
  // On the very first frame, align the yaw angle between the reference trajectory
  // and the robot's actual heading direction
  if (is_first_time_) {
    updateFirstFrameYawAlignment();
  }

  // Update the per-cycle observation context field
  obs_ctx_.policy_step = policy_step;

  int output_offset = 0;
  for (size_t i = 0; i < param_->observation_names.size(); ++i) {
    const std::string& obs_name = param_->observation_names[i];

    // Compute a single-step observation for this component via the registry
    Eigen::VectorXd single = motion_stamp_obs::GetObservation(obs_name, obs_ctx_);

    // Update the sliding-window history buffer for this component
    Eigen::MatrixXd& buf = observation_history_buffers_[i];
    const int hist_len = static_cast<int>(buf.cols());
    if (is_first_time_) {
      // First frame: replicate the current observation across all history steps
      buf.colwise() = single;
    } else {
      // Subsequent frames: shift buffer left by one, insert newest at rightmost column
      if (hist_len > 1) {
        buf.leftCols(hist_len - 1) = buf.rightCols(hist_len - 1).eval();
      }
      buf.rightCols(1) = single;
    }

    // Flatten this component's history buffer (column-major) into the observation vector
    mlp_net_observation_vec.segment(output_offset, buf.size()) =
        Eigen::Map<const Eigen::VectorXd>(buf.data(), buf.size());
    output_offset += buf.size();
  }

  if (is_first_time_) {
    is_first_time_ = false;
  }
}

/**
 * @brief Computes yaw alignment on the first frame.
 *
 * Extracts the yaw angle from both:
 *   - The robot's current IMU orientation (actual heading)
 *   - The reference trajectory's first-frame body orientation
 *
 * These yaw rotations are stored and used by observation functions to transform
 * reference trajectory data into the robot's local coordinate frame. This ensures
 * the dance motion starts in the direction the robot is actually facing, regardless
 * of its initial heading.
 */
void MotionStampRunner::updateFirstFrameYawAlignment() {
  // Get the robot's current orientation from IMU
  const auto imu_info = data_store_->imu_info.Get();
  const auto base_state = data_store_->base_state_in_world.Get();
  Eigen::Matrix3d R_local = math::RotationMatrixd(imu_info->quaternion).matrix();

  // Get the reference trajectory's first-frame body orientation (quaternion: w, x, y, z)
  Eigen::Quaterniond ref_anchor_ori_quat_w(
      (*ref_body_quat_w_all_)(policy_step, 0), (*ref_body_quat_w_all_)(policy_step, 1),
      (*ref_body_quat_w_all_)(policy_step, 2), (*ref_body_quat_w_all_)(policy_step, 3));
  Eigen::Matrix3d ref_anchor_ori_rot_w = math::RotationMatrixd(ref_anchor_ori_quat_w).matrix();

  // Extract yaw angles (rotation about Z-axis) from both orientations
  double ref_yaw = std::atan2(ref_anchor_ori_rot_w(1, 0), ref_anchor_ori_rot_w(0, 0));
  double body_yaw = std::atan2(R_local(1, 0), R_local(0, 0));

  // Store pure yaw rotation matrices for coordinate frame alignment in observations
  ref_init_yaw_rot_ = Eigen::AngleAxisd(ref_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  body_init_yaw_rot_ = Eigen::AngleAxisd(body_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Share with the observation context so observation functions can use them
  obs_ctx_.ref_init_yaw_rot = ref_init_yaw_rot_;
  obs_ctx_.body_init_yaw_rot = body_init_yaw_rot_;
  obs_ctx_.ref_init_pos_w = ref_body_pos_w_all_->row(policy_step).transpose();
  obs_ctx_.body_init_pos_w = base_state->frame.pose.position;

  LOG(INFO) << "[WbtRunner::YawAlignment] policy_step=" << policy_step << ", ref_yaw=" << ref_yaw
            << ", body_yaw=" << body_yaw << ", body_minus_ref_yaw=" << WrapToPi(body_yaw - ref_yaw)
            << ", ref_init_pos_w=[" << obs_ctx_.ref_init_pos_w.transpose() << "]"
            << ", body_init_pos_w=[" << obs_ctx_.body_init_pos_w.transpose() << "]"
            << ", imu_quat_wxyz=[" << imu_info->quaternion.w() << ", " << imu_info->quaternion.x() << ", "
            << imu_info->quaternion.y() << ", " << imu_info->quaternion.z() << "]"
            << ", imu_ang_vel_norm=" << imu_info->angular_velocity.norm()
            << ", base_lin_vel_norm=" << base_state->frame.twist.linear.norm()
            << ", base_ang_vel_norm=" << base_state->frame.twist.angular.norm();
}

// ============================================================================
// Policy Inference and Motor Command
// ============================================================================

/**
 * @brief Runs the MLP policy network inference and computes target joint positions.
 *
 * Pipeline:
 *   1. Read current joint positions and velocities (for state feedback)
 *   2. Forward the assembled observation vector through the MNN model
 *   3. Map the action output to target joint positions:
 *      q_des[active_joints] = ref_joint_pos + action * action_scale
 *
 * @note This matches the tracking training task where the policy outputs a residual
 *       around the reference trajectory joint positions, not an absolute joint target
 *       around the default pose.
 */
void MotionStampRunner::CalculateMotorCommand() {
  // Read current joint state (used internally by some observation functions
  // but NOT directly used in action computation here)
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_real_);

  // Run MLP forward inference (float precision, cast back to double)
  *mlp_net_action_ = (mlp_net_->Inference(mlp_net_observation_vec.cast<float>())).cast<double>();

  // Map action to target joint positions:
  //   q_des = ref_joint_pos + action * action_scale (for policy-controlled joints only)

  q_des_ = *default_joint_q_;
  if (param_->resident_control) {
    const int ref_step = std::min(policy_step, max_policy_step);
    const Eigen::VectorXd ref_joint_pos = ref_joint_pos_all_->row(ref_step);
    const Eigen::VectorXd scaled_action = mlp_net_action_->cwiseProduct(action_scale_);
    q_des_(*policy2deploy_joint_idx_) = ref_joint_pos + scaled_action;
  } else {
    q_des_(*policy2deploy_joint_idx_) += mlp_net_action_->cwiseProduct(action_scale_);
  }

  if (log_first_policy_command_) {
    const Eigen::VectorXd active_q = q_real_(*policy2deploy_joint_idx_);
    const Eigen::VectorXd active_q_des = q_des_(*policy2deploy_joint_idx_);
    const Eigen::VectorXd active_err = active_q_des - active_q;
    LOG(INFO) << "[WbtRunner::FirstPolicyCommand] policy_step=" << policy_step
              << ", obs_norm=" << mlp_net_observation_vec.norm()
              << ", obs_min=" << mlp_net_observation_vec.minCoeff()
              << ", obs_max=" << mlp_net_observation_vec.maxCoeff()
              << ", action_norm=" << mlp_net_action_->norm()
              << ", action_min=" << mlp_net_action_->minCoeff()
              << ", action_max=" << mlp_net_action_->maxCoeff()
              << ", active_q_err_norm=" << active_err.norm()
              << ", active_q_err_max_abs=" << active_err.cwiseAbs().maxCoeff();
    log_first_policy_command_ = false;
  }
}

/**
 * @brief Sends computed target positions to the motor controllers via PD control.
 *
 * Sets target velocity and feedforward torque to zero (pure PD position control).
 * The low-level driver computes: tau = kp*(q_des-q) + kd*(0-qd) + 0
 */
void MotionStampRunner::SendMotorCommand() {
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);
}

// ============================================================================
// Runner Exit Logic
// ============================================================================

/**
 * @brief Immediately allows exit (no graceful transition needed for dance playback).
 */
TransitionState MotionStampRunner::TryExit() { return TransitionState::kCompleted; }

/**
 * @brief Post-exit cleanup. Currently no additional actions needed.
 */
bool MotionStampRunner::Exit() { return true; }

/**
 * @brief Runner termination. Currently no additional actions needed.
 */
void MotionStampRunner::End() {}

// ============================================================================
// Utility Methods
// ============================================================================

/**
 * @brief Initializes the per-observation-component history buffers.
 *
 * Each observation component in `observation_names` gets its own history buffer
 * with dimensions [component_dim x history_length]. The history length is read
 * from `observation_history_lengths`, defaulting to 1 (no history) if not specified.
 *
 * For example, if observation "joint_pos" has dim=12 and history_length=3,
 * its buffer will be a 12x3 matrix, flattening to 36 elements in the final
 * observation vector.
 */
void MotionStampRunner::initHistoryBuffers() {
  observation_history_buffers_.clear();
  observation_history_buffers_.reserve(param_->observation_names.size());
  for (size_t i = 0; i < param_->observation_names.size(); ++i) {
    int dim = GetObservationDim(param_->observation_names[i]);
    int hist_len = (i < param_->observation_history_lengths.size()) ? param_->observation_history_lengths[i] : 1;
    observation_history_buffers_.emplace_back(dim, hist_len);
    observation_history_buffers_.back().setZero();
  }
}

Eigen::MatrixXd MotionStampRunner::LoadTrajectoryMatrix(std::initializer_list<std::string_view> candidate_keys,
                                                        int row_index) {
  for (std::string_view key : candidate_keys) {
    auto it = trajectory_npz.find(std::string(key));
    if (it != trajectory_npz.end()) {
      return npyFloatToMatrixXd(it->second, row_index);
    }
  }

  std::ostringstream oss;
  bool first = true;
  for (std::string_view key : candidate_keys) {
    if (!first) oss << ", ";
    oss << key;
    first = false;
  }
  throw std::runtime_error("trajectory missing required key; expected one of: " + oss.str());
}

/**
 * @brief Converts a cnpy NpyArray (float) to an Eigen MatrixXd (double).
 *
 * Supports two array shapes:
 *   - 2D array [rows x cols]: directly converted to MatrixXd
 *   - 3D array [dim0 x dim1 x dim2]: extracts a 2D slice at the given row_index
 *     along dim1, producing a [dim0 x dim2] matrix
 *
 * @param npy_array The numpy array loaded from .npz file
 * @param row_index For 3D arrays, the index along the second dimension to extract
 * @return Eigen::MatrixXd containing the converted data
 * @throws std::runtime_error if array dimensions are not 2D or 3D, or if row_index is invalid
 */
Eigen::MatrixXd MotionStampRunner::npyFloatToMatrixXd(const cnpy::NpyArray& npy_array, int row_index) {
  const std::vector<size_t>& shape = npy_array.shape;
  const float* data = npy_array.data<float>();
  if (shape.size() == 2) {
    size_t rows = shape[0], cols = shape[1];
    Eigen::MatrixXd mat(rows, cols);
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j) mat(i, j) = static_cast<double>(data[i * cols + j]);
    return mat;
  }
  if (shape.size() == 3) {
    size_t dim0 = shape[0], dim1 = shape[1], dim2 = shape[2];
    if (row_index < 0 || static_cast<size_t>(row_index) >= dim1)
      throw std::runtime_error("Invalid row_index: " + std::to_string(row_index));
    Eigen::MatrixXd mat(dim0, dim2);
    for (size_t d0 = 0; d0 < dim0; ++d0)
      for (size_t d2 = 0; d2 < dim2; ++d2)
        mat(d0, d2) = static_cast<double>(data[d0 * (dim1 * dim2) + row_index * dim2 + d2]);
    return mat;
  }
  throw std::runtime_error("Unsupported array dimension: " + std::to_string(shape.size()));
}

Eigen::MatrixXd MotionStampRunner::ReorderTrajectoryJointColumns(const Eigen::MatrixXd& matrix,
                                                                    std::string_view matrix_name) const {
  if (param_->trajectory_joint_names) {
    const std::vector<std::string>& trajectory_joint_names = *param_->trajectory_joint_names;
    if (trajectory_joint_names.size() != static_cast<size_t>(matrix.cols())) {
      throw std::runtime_error(std::string(matrix_name) + " column count does not match trajectory_joint_names size");
    }

    std::unordered_map<std::string, int> trajectory_joint_index;
    trajectory_joint_index.reserve(trajectory_joint_names.size());
    for (size_t i = 0; i < trajectory_joint_names.size(); ++i) {
      trajectory_joint_index.emplace(trajectory_joint_names[i], static_cast<int>(i));
    }

    Eigen::MatrixXd reordered(matrix.rows(), param_->num_actions);
    for (size_t i = 0; i < param_->joint_names.size(); ++i) {
      auto it = trajectory_joint_index.find(param_->joint_names[i]);
      if (it == trajectory_joint_index.end()) {
        throw std::runtime_error(std::string(matrix_name) + " is missing trajectory joint: " + param_->joint_names[i]);
      }
      reordered.col(static_cast<int>(i)) = matrix.col(it->second);
    }
    return reordered;
  }

  if (matrix.cols() != param_->num_actions) {
    throw std::runtime_error(std::string(matrix_name) + " column count does not match num_actions");
  }
  return matrix;
}

/**
 * @brief Returns the dimension of a named observation component.
 * @param name The observation component name (as registered in motion_stamp_obs)
 * @return The number of elements in a single-step observation for this component
 */
int MotionStampRunner::GetObservationDim(const std::string& name) const {
  return motion_stamp_obs::GetObservationDim(name, param_->num_actions);
}

/**
 * @brief Computes the total flattened observation vector dimension.
 *
 * Sums (component_dim × history_length) for all observation components.
 * This determines the input dimension of the policy network.
 *
 * @return Total number of elements in the assembled observation vector
 */
int MotionStampRunner::ComputeTotalObservationDim() const {
  int total = 0;
  for (size_t i = 0; i < param_->observation_names.size(); ++i) {
    int hist_len = (i < param_->observation_history_lengths.size()) ? param_->observation_history_lengths[i] : 1;
    total += GetObservationDim(param_->observation_names[i]) * hist_len;
  }
  return total;
}

}  // namespace runner
