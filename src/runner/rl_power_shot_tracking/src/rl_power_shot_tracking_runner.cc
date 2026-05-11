#include "rl_power_shot_tracking/rl_power_shot_tracking_runner.h"

#include <MNN/ErrorCode.hpp>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <glog/logging.h>

#include "parameter/global_config_initializer.h"
#include "tool/string_join.h"

namespace runner {
namespace {

constexpr std::array<std::string_view, 10> kExpectedObservationNames = {
    "command",
    "motion_anchor_pos_b",
    "motion_anchor_ori_b",
    "base_ang_vel",
    "joint_pos",
    "joint_vel",
    "actions",
    "projected_gravity",
    "joint_error",
    "motion_phase",
};

constexpr int kAnchorBodyIndex = 0;
constexpr int kRotationColumnsRows = 3;
constexpr int kRotationColumnsCols = 2;
constexpr float kTwoPi = 6.2831853071795864769F;

int ElementCount(MNN::Tensor *tensor) {
  if (!tensor)
    return 0;
  return tensor->elementSize();
}

void ValidateTensorSize(MNN::Tensor *tensor, int expected_size,
                        const std::string &name) {
  if (!tensor) {
    throw std::runtime_error("MNN tensor is missing: " + name);
  }
  const int actual_size = ElementCount(tensor);
  if (actual_size != expected_size) {
    throw std::runtime_error("Unexpected MNN tensor size for " + name +
                             ": expected " + std::to_string(expected_size) +
                             ", got " + std::to_string(actual_size));
  }
}

void CopyVectorToTensor(MNN::Tensor *tensor, const Eigen::VectorXf &value,
                        const std::string &name) {
  ValidateTensorSize(tensor, static_cast<int>(value.size()), name);
  MNN::Tensor host(tensor, tensor->getDimensionType());
  std::copy(value.data(), value.data() + value.size(), host.host<float>());
  if (!tensor->copyFromHostTensor(&host)) {
    throw std::runtime_error("Failed to copy host data into MNN tensor: " +
                             name);
  }
}

Eigen::VectorXf CopyTensorToVector(MNN::Tensor *tensor,
                                   const std::string &name) {
  if (!tensor) {
    throw std::runtime_error("MNN tensor is missing: " + name);
  }
  MNN::Tensor host(tensor, tensor->getDimensionType());
  if (!tensor->copyToHostTensor(&host)) {
    throw std::runtime_error("Failed to copy MNN tensor to host: " + name);
  }

  Eigen::VectorXf value(host.elementSize());
  std::copy(host.host<float>(), host.host<float>() + host.elementSize(),
            value.data());
  return value;
}

Eigen::MatrixXf VectorToRowMajorMatrix(const Eigen::VectorXf &value, int rows,
                                       int cols, const std::string &name) {
  if (value.size() != rows * cols) {
    throw std::runtime_error("Unexpected matrix element count for " + name);
  }

  Eigen::MatrixXf matrix(rows, cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      matrix(row, col) = value(row * cols + col);
    }
  }
  return matrix;
}

Eigen::VectorXf FlattenRotationFirstTwoColumns(const Eigen::Quaternionf &quat) {
  Eigen::Matrix3f rot = quat.normalized().toRotationMatrix();
  Eigen::VectorXf out(kRotationColumnsRows * kRotationColumnsCols);
  int cursor = 0;
  for (int row = 0; row < kRotationColumnsRows; ++row) {
    for (int col = 0; col < kRotationColumnsCols; ++col) {
      out(cursor++) = rot(row, col);
    }
  }
  return out;
}

bool LinkStateLooksInitialized(const data::LinkInfo &link) {
  const auto &frame = link.frame;
  return frame.pose.position.norm() > 1.0e-6 ||
         frame.twist.linear.norm() > 1.0e-6 ||
         frame.twist.angular.norm() > 1.0e-6 ||
         std::abs(frame.pose.quaternion.w() - 1.0) > 1.0e-6 ||
         frame.pose.quaternion.vec().norm() > 1.0e-6;
}

Eigen::Quaternionf RowToQuaternion(const Eigen::MatrixXf &matrix, int row) {
  Eigen::Quaternionf quat(matrix(row, 0), matrix(row, 1), matrix(row, 2),
                          matrix(row, 3));
  quat.normalize();
  return quat;
}

Eigen::Vector3f RowToVector3(const Eigen::MatrixXf &matrix, int row) {
  return Eigen::Vector3f(matrix(row, 0), matrix(row, 1), matrix(row, 2));
}

Eigen::VectorXf BuildMotionPhase(int time_step, int time_step_total) {
  Eigen::VectorXf motion_phase(2);
  if (time_step_total <= 0) {
    motion_phase << 0.0F, 1.0F;
    return motion_phase;
  }

  const int wrapped_step = ((time_step % time_step_total) + time_step_total) %
                           time_step_total;
  const float phase = kTwoPi * static_cast<float>(wrapped_step) /
                      static_cast<float>(time_step_total);
  motion_phase << std::sin(phase), std::cos(phase);
  return motion_phase;
}

} // namespace

struct RlPowerShotTrackingRunner::ReferenceState {
  Eigen::VectorXf joint_pos;
  Eigen::VectorXf joint_vel;
  Eigen::MatrixXf body_pos_w;
  Eigen::MatrixXf body_quat_w;
  Eigen::MatrixXf body_lin_vel_w;
  Eigen::MatrixXf body_ang_vel_w;
};

class RlPowerShotTrackingRunner::TrackingPolicyModel {
public:
  TrackingPolicyModel(const std::string &model_path, int obs_dim,
                      int num_actions) {
    if (!std::filesystem::exists(model_path)) {
      throw std::runtime_error("Power Shot policy file does not exist: " +
                               model_path);
    }

    net_.reset(MNN::Interpreter::createFromFile(model_path.c_str()));
    if (!net_) {
      throw std::runtime_error("Failed to create MNN interpreter for: " +
                               model_path);
    }

    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_CPU;
    config.numThread = 1;
    session_ = net_->createSession(config);
    if (!session_) {
      throw std::runtime_error("Failed to create MNN session for: " +
                               model_path);
    }

    obs_tensor_ = net_->getSessionInput(session_, "obs");
    time_step_tensor_ = net_->getSessionInput(session_, "time_step");
    actions_tensor_ = net_->getSessionOutput(session_, "actions");
    joint_pos_tensor_ = net_->getSessionOutput(session_, "joint_pos");
    joint_vel_tensor_ = net_->getSessionOutput(session_, "joint_vel");
    body_pos_w_tensor_ = net_->getSessionOutput(session_, "body_pos_w");
    body_quat_w_tensor_ = net_->getSessionOutput(session_, "body_quat_w");
    body_lin_vel_w_tensor_ = net_->getSessionOutput(session_, "body_lin_vel_w");
    body_ang_vel_w_tensor_ = net_->getSessionOutput(session_, "body_ang_vel_w");

    ValidateTensorSize(obs_tensor_, obs_dim, "obs");
    ValidateTensorSize(time_step_tensor_, 1, "time_step");
    ValidateTensorSize(actions_tensor_, num_actions, "actions");
    ValidateTensorSize(joint_pos_tensor_, num_actions, "joint_pos");
    ValidateTensorSize(joint_vel_tensor_, num_actions, "joint_vel");
    ValidateTensorSize(body_pos_w_tensor_, 9, "body_pos_w");
    ValidateTensorSize(body_quat_w_tensor_, 12, "body_quat_w");
    ValidateTensorSize(body_lin_vel_w_tensor_, 9, "body_lin_vel_w");
    ValidateTensorSize(body_ang_vel_w_tensor_, 9, "body_ang_vel_w");
  }

  Eigen::VectorXf RunAction(const Eigen::VectorXf &obs, int time_step) {
    RunSession(obs, time_step);
    return CopyTensorToVector(actions_tensor_, "actions");
  }

  ReferenceState RunReference(const Eigen::VectorXf &zero_obs, int time_step) {
    RunSession(zero_obs, time_step);
    ReferenceState reference;
    reference.joint_pos = CopyTensorToVector(joint_pos_tensor_, "joint_pos");
    reference.joint_vel = CopyTensorToVector(joint_vel_tensor_, "joint_vel");
    reference.body_pos_w = VectorToRowMajorMatrix(
        CopyTensorToVector(body_pos_w_tensor_, "body_pos_w"), 3, 3,
        "body_pos_w");
    reference.body_quat_w = VectorToRowMajorMatrix(
        CopyTensorToVector(body_quat_w_tensor_, "body_quat_w"), 3, 4,
        "body_quat_w");
    reference.body_lin_vel_w = VectorToRowMajorMatrix(
        CopyTensorToVector(body_lin_vel_w_tensor_, "body_lin_vel_w"), 3, 3,
        "body_lin_vel_w");
    reference.body_ang_vel_w = VectorToRowMajorMatrix(
        CopyTensorToVector(body_ang_vel_w_tensor_, "body_ang_vel_w"), 3, 3,
        "body_ang_vel_w");
    return reference;
  }

private:
  void RunSession(const Eigen::VectorXf &obs, int time_step) {
    Eigen::VectorXf time_step_value(1);
    time_step_value(0) = static_cast<float>(time_step);
    CopyVectorToTensor(obs_tensor_, obs, "obs");
    CopyVectorToTensor(time_step_tensor_, time_step_value, "time_step");

    MNN::ErrorCode code = net_->runSession(session_);
    if (code != MNN::NO_ERROR) {
      throw std::runtime_error("MNN runSession failed, error code: " +
                               std::to_string(static_cast<int>(code)));
    }
  }

  std::shared_ptr<MNN::Interpreter> net_;
  MNN::Session *session_ = nullptr;
  MNN::Tensor *obs_tensor_ = nullptr;
  MNN::Tensor *time_step_tensor_ = nullptr;
  MNN::Tensor *actions_tensor_ = nullptr;
  MNN::Tensor *joint_pos_tensor_ = nullptr;
  MNN::Tensor *joint_vel_tensor_ = nullptr;
  MNN::Tensor *body_pos_w_tensor_ = nullptr;
  MNN::Tensor *body_quat_w_tensor_ = nullptr;
  MNN::Tensor *body_lin_vel_w_tensor_ = nullptr;
  MNN::Tensor *body_ang_vel_w_tensor_ = nullptr;
};

RlPowerShotTrackingRunner::~RlPowerShotTrackingRunner() = default;

void RlPowerShotTrackingRunner::SetupContext() {
  data_store_->parallel_by_classic_parser.store(false);
}

void RlPowerShotTrackingRunner::TeardownContext() {}

bool RlPowerShotTrackingRunner::Enter() {
  if (!param_tag_.empty()) {
    param_ =
        data::ParamManager::create<data::RlPowerShotTrackingParam>(param_tag_);
  }
  if (!param_) {
    LOG(ERROR) << "[RlPowerShotTrackingRunner::Enter] Failed to load parameters";
    return false;
  }
  param_->num_actions = static_cast<int>(param_->joint_names.size());

  try {
    InitializeJointMapping();
    InitializeObservationBuffers();

    const int obs_dim = ComputeObservationDim();
    zero_observation_ = Eigen::VectorXf::Zero(obs_dim);
    last_action_policy_ = Eigen::VectorXf::Zero(param_->num_actions);

    const std::string policy_path = common::PathJoin(
        common::GlobalPathManager::GetInstance().GetConfigPath(),
        param_->policy_file);
    policy_ = std::make_unique<TrackingPolicyModel>(policy_path, obs_dim,
                                                    param_->num_actions);

    ReadCurrentState();
    InitializeReferenceAlignment();
    q_enter_ = q_real_;
    q_des_ = q_real_;
    qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
    tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
    GetMutableOutput().Reset();
    GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_,
                                  tau_ff_des_);

    time_step_ = 0;
    iter_ = 0;
    finished_ = false;
    policy_started_ = false;
  } catch (const std::exception &error) {
    LOG(ERROR) << "[RlPowerShotTrackingRunner::Enter] " << error.what();
    return false;
  }

  LOG(INFO) << "[RlPowerShotTrackingRunner::Enter] Loaded Power Shot policy, "
            << "obs_dim=" << zero_observation_.size()
            << ", actions=" << param_->num_actions
            << ", time_step_total=" << param_->time_step_total;
  return true;
}

void RlPowerShotTrackingRunner::Run() {
  if (!policy_) {
    LOG(ERROR) << "[RlPowerShotTrackingRunner::Run] Policy is not loaded";
    return;
  }

  data_store_->reference_contact_signal_info->SetAllLegContactSignal();
  data_store_->reference_contact_signal_info->SetAllArmNonContactSignal();

  try {
    ReadCurrentState();
    ReferenceState reference = SampleReference(time_step_);

    if (!policy_started_) {
      ResetObservationBuffers();
      last_action_policy_.setZero();
      policy_started_ = true;
    }

    Eigen::VectorXf obs = BuildObservation(reference);
    Eigen::VectorXf action = policy_->RunAction(obs, time_step_);
    if (iter_ < 5 || iter_ % 50 == 0) {
      LOG(INFO) << "[DEBUG-POWER-SHOT] iter=" << iter_
                << " reference_time_step=" << time_step_
                << " action_time_step=" << time_step_
                << " obs_norm=" << obs.norm()
                << " action_min=" << action.minCoeff()
                << " action_max=" << action.maxCoeff()
                << " action_norm=" << action.norm();
    }
    CalculateMotorCommand(action);
    SendMotorCommand();
    last_action_policy_ = action;
    AdvanceTimeStep();
  } catch (const std::exception &error) {
    LOG(ERROR) << "[RlPowerShotTrackingRunner::Run] " << error.what();
    SetRunnerState(RunnerState::kTryExit);
  }
}

TransitionState RlPowerShotTrackingRunner::TryExit() {
  return TransitionState::kCompleted;
}

bool RlPowerShotTrackingRunner::Exit() { return true; }

void RlPowerShotTrackingRunner::End() {}

void RlPowerShotTrackingRunner::InitializeJointMapping() {
  const int num_actions = param_->num_actions;
  if (num_actions <= 0) {
    throw std::runtime_error("joint_names is empty");
  }
  if (param_->joint_stiffness.size() != num_actions ||
      param_->joint_damping.size() != num_actions ||
      param_->default_joint_pos.size() != num_actions ||
      param_->action_scale.size() != num_actions) {
    throw std::runtime_error(
        "Power Shot joint parameter vector sizes must match joint_names");
  }

  policy2deploy_joint_idx_.setZero(num_actions);
  for (int i = 0; i < num_actions; ++i) {
    const auto &joint_name = param_->joint_names.at(static_cast<size_t>(i));
    if (!model_param_->joint_id_in_total_limb.contains(joint_name)) {
      throw std::runtime_error("Unknown SDK joint name in Power Shot config: " +
                               joint_name);
    }
    policy2deploy_joint_idx_(i) =
        model_param_->joint_id_in_total_limb.at(joint_name);
  }

  default_joint_q_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  joint_kp_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  joint_kd_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  default_joint_pos_policy_ = param_->default_joint_pos;
  action_scale_policy_ = param_->action_scale;

  default_joint_q_(policy2deploy_joint_idx_) = default_joint_pos_policy_;
  joint_kp_(policy2deploy_joint_idx_) = param_->joint_stiffness;
  joint_kd_(policy2deploy_joint_idx_) = param_->joint_damping;
}

void RlPowerShotTrackingRunner::InitializeObservationBuffers() {
  if (param_->observation_names.size() != kExpectedObservationNames.size()) {
    throw std::runtime_error(
        "Power Shot observation_names must contain exactly 10 terms");
  }
  for (size_t i = 0; i < kExpectedObservationNames.size(); ++i) {
    if (param_->observation_names[i] != kExpectedObservationNames[i]) {
      throw std::runtime_error(
          "Unsupported Power Shot observation layout at index " +
          std::to_string(i) + ": expected " +
          std::string(kExpectedObservationNames[i]) + ", got " +
          param_->observation_names[i]);
    }
  }

  if (param_->observation_history_lengths.size() !=
      param_->observation_names.size()) {
    throw std::runtime_error(
        "observation_history_lengths must match observation_names");
  }

  observation_histories_.clear();
  observation_histories_.reserve(param_->observation_names.size());
  int single_frame_dim = 0;
  for (size_t i = 0; i < param_->observation_names.size(); ++i) {
    ObservationHistory history;
    history.history_length = param_->observation_history_lengths[i];
    history.value_dim = GetObservationDim(param_->observation_names[i]);
    if (history.history_length < 1 || history.value_dim < 1) {
      throw std::runtime_error("Invalid observation history config for " +
                               param_->observation_names[i]);
    }
    history.values =
        Eigen::VectorXf::Zero(history.history_length * history.value_dim);
    single_frame_dim += history.value_dim;
    observation_histories_.push_back(std::move(history));
  }
  if (single_frame_dim != 161) {
    throw std::runtime_error(
        "Unexpected Power Shot single-frame observation dim: " +
        std::to_string(single_frame_dim));
  }
}

void RlPowerShotTrackingRunner::InitializeReferenceAlignment() {
  reference_alignment_rot_.setIdentity();
  reference_alignment_quat_.setIdentity();
  reference_alignment_pos_.setZero();
  reference_alignment_initialized_ = false;

  if (!param_->align_reference_to_robot_anchor) {
    return;
  }

  ReferenceState initial_reference = policy_->RunReference(zero_observation_, 0);
  const data::LinkInfo robot_anchor = GetRobotAnchorState();
  const Eigen::Vector3f robot_anchor_pos =
      robot_anchor.frame.pose.position.cast<float>();
  Eigen::Quaternionf robot_anchor_quat(
      static_cast<float>(robot_anchor.frame.pose.quaternion.w()),
      static_cast<float>(robot_anchor.frame.pose.quaternion.x()),
      static_cast<float>(robot_anchor.frame.pose.quaternion.y()),
      static_cast<float>(robot_anchor.frame.pose.quaternion.z()));
  robot_anchor_quat.normalize();

  const Eigen::Vector3f reference_anchor_pos =
      RowToVector3(initial_reference.body_pos_w, kAnchorBodyIndex);
  const Eigen::Quaternionf reference_anchor_quat =
      RowToQuaternion(initial_reference.body_quat_w, kAnchorBodyIndex);

  reference_alignment_quat_ =
      robot_anchor_quat * reference_anchor_quat.conjugate();
  reference_alignment_quat_.normalize();
  reference_alignment_rot_ = reference_alignment_quat_.toRotationMatrix();
  reference_alignment_pos_ =
      robot_anchor_pos - reference_alignment_rot_ * reference_anchor_pos;
  reference_alignment_initialized_ = true;

  LOG(INFO) << "[RlPowerShotTrackingRunner::InitializeReferenceAlignment] "
            << "reference anchor aligned to robot anchor, translation=("
            << reference_alignment_pos_.transpose() << ")";
}

void RlPowerShotTrackingRunner::ResetObservationBuffers() {
  for (auto &history : observation_histories_) {
    history.values.setZero();
    history.initialized = false;
  }
}

void RlPowerShotTrackingRunner::ReadCurrentState() {
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_real_);
}

RlPowerShotTrackingRunner::ReferenceState
RlPowerShotTrackingRunner::SampleReference(int time_step) {
  ReferenceState reference = policy_->RunReference(zero_observation_, time_step);
  ApplyReferenceAlignment(reference);
  return reference;
}

void RlPowerShotTrackingRunner::ApplyReferenceAlignment(
    ReferenceState &reference) const {
  if (!reference_alignment_initialized_) {
    return;
  }

  for (int row = 0; row < reference.body_pos_w.rows(); ++row) {
    const Eigen::Vector3f pos = RowToVector3(reference.body_pos_w, row);
    const Eigen::Vector3f aligned_pos =
        reference_alignment_rot_ * pos + reference_alignment_pos_;
    reference.body_pos_w.row(row) = aligned_pos.transpose();

    Eigen::Quaternionf quat =
        reference_alignment_quat_ * RowToQuaternion(reference.body_quat_w, row);
    quat.normalize();
    reference.body_quat_w(row, 0) = quat.w();
    reference.body_quat_w(row, 1) = quat.x();
    reference.body_quat_w(row, 2) = quat.y();
    reference.body_quat_w(row, 3) = quat.z();

    const Eigen::Vector3f lin_vel = RowToVector3(reference.body_lin_vel_w, row);
    reference.body_lin_vel_w.row(row) =
        (reference_alignment_rot_ * lin_vel).transpose();

    const Eigen::Vector3f ang_vel = RowToVector3(reference.body_ang_vel_w, row);
    reference.body_ang_vel_w.row(row) =
        (reference_alignment_rot_ * ang_vel).transpose();
  }
}

Eigen::VectorXf
RlPowerShotTrackingRunner::BuildObservation(const ReferenceState &reference) {
  const data::LinkInfo base = GetRobotAnchorState();
  Eigen::Quaterniond robot_anchor_quat =
      base.frame.pose.quaternion.normalized();
  Eigen::Matrix3d robot_anchor_rot = robot_anchor_quat.toRotationMatrix();

  Eigen::VectorXf command(reference.joint_pos.size() +
                          reference.joint_vel.size());
  command << reference.joint_pos, reference.joint_vel;

  Eigen::Vector3f reference_anchor_pos =
      RowToVector3(reference.body_pos_w, kAnchorBodyIndex);
  Eigen::Quaternionf reference_anchor_quat =
      RowToQuaternion(reference.body_quat_w, kAnchorBodyIndex);
  Eigen::Vector3f robot_anchor_pos = base.frame.pose.position.cast<float>();
  Eigen::Matrix3f robot_anchor_rot_f = robot_anchor_rot.cast<float>();

  Eigen::Vector3f motion_anchor_pos_b =
      robot_anchor_rot_f.transpose() *
      (reference_anchor_pos - robot_anchor_pos);
  Eigen::Quaternionf robot_anchor_quat_f(
      static_cast<float>(robot_anchor_quat.w()),
      static_cast<float>(robot_anchor_quat.x()),
      static_cast<float>(robot_anchor_quat.y()),
      static_cast<float>(robot_anchor_quat.z()));
  Eigen::Quaternionf motion_anchor_quat_b =
      robot_anchor_quat_f.conjugate() * reference_anchor_quat;
  Eigen::VectorXf motion_anchor_ori_b =
      FlattenRotationFirstTwoColumns(motion_anchor_quat_b);

  Eigen::Vector3f base_ang_vel =
      (robot_anchor_rot.transpose() * base.frame.twist.angular).cast<float>();
  Eigen::Vector3f projected_gravity =
      (-robot_anchor_rot.transpose() * Eigen::Vector3d::UnitZ()).cast<float>();

  if (iter_ < 5 || iter_ % 50 == 0) {
    LOG(INFO) << "[DEBUG-POWER-SHOT] iter=" << iter_
              << " robot_anchor_pos=(" << robot_anchor_pos.transpose() << ")"
              << " reference_anchor_pos=(" << reference_anchor_pos.transpose()
              << ") motion_anchor_pos_b=(" << motion_anchor_pos_b.transpose()
              << ") base_ang_vel=(" << base_ang_vel.transpose()
              << ") projected_gravity=(" << projected_gravity.transpose()
              << ")";
  }

  Eigen::VectorXf robot_joint_pos =
      q_real_(policy2deploy_joint_idx_).cast<float>();
  Eigen::VectorXf joint_pos =
      (robot_joint_pos.cast<double>() - default_joint_pos_policy_).cast<float>();
  Eigen::VectorXf joint_vel = qd_real_(policy2deploy_joint_idx_).cast<float>();
  Eigen::VectorXf joint_error = reference.joint_pos - robot_joint_pos;
  Eigen::VectorXf motion_phase =
      BuildMotionPhase(time_step_, param_->time_step_total);

  std::array<Eigen::VectorXf, 10> terms = {
      command,      motion_anchor_pos_b, motion_anchor_ori_b, base_ang_vel,
      joint_pos,    joint_vel,           last_action_policy_, projected_gravity,
      joint_error,  motion_phase,
  };

  Eigen::VectorXf obs(ComputeObservationDim());
  int cursor = 0;
  for (size_t i = 0; i < terms.size(); ++i) {
    const int expected_dim = GetObservationDim(param_->observation_names[i]);
    if (terms[i].size() != expected_dim) {
      throw std::runtime_error("Observation term " +
                               param_->observation_names[i] + " has size " +
                               std::to_string(terms[i].size()) +
                               ", expected " + std::to_string(expected_dim));
    }
    Eigen::VectorXf processed = ProcessObservationTerm(i, terms[i]);
    obs.segment(cursor, processed.size()) = processed;
    cursor += static_cast<int>(processed.size());
  }
  return obs;
}

Eigen::VectorXf
RlPowerShotTrackingRunner::ProcessObservationTerm(size_t term_index,
                                                  const Eigen::VectorXf &value) {
  if (term_index >= observation_histories_.size()) {
    throw std::runtime_error("Observation term index out of range");
  }

  ObservationHistory &history = observation_histories_[term_index];
  if (value.size() != history.value_dim) {
    throw std::runtime_error("Observation term " +
                             param_->observation_names[term_index] +
                             " has size " + std::to_string(value.size()) +
                             ", expected " + std::to_string(history.value_dim));
  }

  if (!history.initialized) {
    for (int i = 0; i < history.history_length; ++i) {
      history.values.segment(i * history.value_dim, history.value_dim) = value;
    }
    history.initialized = true;
  } else {
    const int shift_size = (history.history_length - 1) * history.value_dim;
    if (shift_size > 0) {
      history.values.segment(0, shift_size) =
          history.values.segment(history.value_dim, shift_size).eval();
    }
    history.values.tail(history.value_dim) = value;
  }

  return history.values;
}

int RlPowerShotTrackingRunner::GetObservationDim(const std::string &name) const {
  if (name == "command")
    return 2 * param_->num_actions;
  if (name == "motion_anchor_pos_b")
    return 3;
  if (name == "motion_anchor_ori_b")
    return 6;
  if (name == "base_ang_vel")
    return 3;
  if (name == "projected_gravity")
    return 3;
  if (name == "joint_pos")
    return param_->num_actions;
  if (name == "joint_vel")
    return param_->num_actions;
  if (name == "actions")
    return param_->num_actions;
  if (name == "joint_error")
    return param_->num_actions;
  if (name == "motion_phase")
    return 2;
  return 0;
}

int RlPowerShotTrackingRunner::ComputeObservationDim() const {
  int dim = 0;
  for (size_t i = 0; i < param_->observation_names.size(); ++i) {
    dim += GetObservationDim(param_->observation_names[i]) *
           param_->observation_history_lengths[i];
  }
  return dim;
}

void RlPowerShotTrackingRunner::CalculateWarmupMotorCommand(
    const ReferenceState &reference) {
  Eigen::VectorXd target_policy = reference.joint_pos.cast<double>();
  q_des_ = q_real_;
  q_des_(policy2deploy_joint_idx_) = target_policy;

  const double elapsed = static_cast<double>(iter_) * runner_period_;
  const double alpha =
      std::clamp(elapsed / param_->transition_duration_s, 0.0, 1.0);
  q_des_ = alpha * q_des_ + (1.0 - alpha) * q_enter_;

  if (iter_ < 5 || iter_ % 50 == 0) {
    LOG(INFO) << "[DEBUG-POWER-SHOT] warmup iter=" << iter_
              << " alpha=" << alpha
              << " target_ref_min=" << target_policy.minCoeff()
              << " target_ref_max=" << target_policy.maxCoeff();
  }
}

void RlPowerShotTrackingRunner::CalculateMotorCommand(
    const Eigen::VectorXf &action) {
  if (action.size() != param_->num_actions) {
    throw std::runtime_error("Power Shot policy action size mismatch");
  }

  Eigen::VectorXd target_policy =
      default_joint_pos_policy_ +
      action_scale_policy_.cwiseProduct(action.cast<double>());
  q_des_ = q_real_;
  q_des_(policy2deploy_joint_idx_) = target_policy;

  const double elapsed = static_cast<double>(iter_) * runner_period_;
  if (param_->transition_duration_s > 0.0 &&
      elapsed < param_->transition_duration_s) {
    const double alpha =
        std::clamp(elapsed / param_->transition_duration_s, 0.0, 1.0);
    q_des_ = alpha * q_des_ + (1.0 - alpha) * q_enter_;
  }
}

void RlPowerShotTrackingRunner::SendMotorCommand() {
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_,
                                tau_ff_des_);
}

void RlPowerShotTrackingRunner::AdvanceTimeStep() {
  ++iter_;

  if (param_->time_step_total <= 0) {
    ++time_step_;
    return;
  }

  if (time_step_ + 1 < param_->time_step_total) {
    ++time_step_;
    return;
  }

  if (param_->loop_motion) {
    time_step_ = 0;
    if (param_->reset_observation_history_on_loop) {
      ResetObservationBuffers();
      last_action_policy_.setZero();
    }
    return;
  }

  time_step_ = param_->time_step_total - 1;
  if (!finished_) {
    finished_ = true;
    if (param_->auto_transition) {
      SetRunnerState(RunnerState::kTryExit);
    }
  }
}

data::LinkInfo RlPowerShotTrackingRunner::GetRobotAnchorState() const {
  data::LinkInfo simulated_base =
      *data_store_->simulated_base_state_in_world.Get();
  if (LinkStateLooksInitialized(simulated_base)) {
    return simulated_base;
  }

  data::LinkInfo estimated_base = *data_store_->base_state_in_world.Get();
  if (LinkStateLooksInitialized(estimated_base)) {
    return estimated_base;
  }

  data::LinkInfo fallback;
  auto imu = data_store_->imu_info.Get();
  fallback.frame.pose.quaternion = imu->quaternion;
  fallback.frame.twist.angular = imu->angular_velocity;
  return fallback;
}

} // namespace runner
