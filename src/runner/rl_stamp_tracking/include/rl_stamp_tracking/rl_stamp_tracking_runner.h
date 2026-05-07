#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "rl_stamp_tracking_param/rl_stamp_tracking_param.h"

namespace runner {

class RlStampTrackingRunner : public MotionRunner {
public:
  RlStampTrackingRunner(std::string_view name,
                        const std::shared_ptr<data::DataStore> &data_store)
      : MotionRunner(name, data_store) {
    param_ = data::ParamManager::create<data::RlStampTrackingParam>();
  }
  ~RlStampTrackingRunner() override;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

private:
  struct ReferenceState;
  class TrackingPolicyModel;

  struct ObservationHistory {
    Eigen::VectorXf values;
    int history_length = 1;
    int value_dim = 0;
    bool initialized = false;
  };

  void InitializeJointMapping();
  void InitializeObservationBuffers();
  void InitializeReferenceAlignment();
  void ResetObservationBuffers();
  void ReadCurrentState();
  ReferenceState SampleReference(int time_step);
  void ApplyReferenceAlignment(ReferenceState &reference) const;
  Eigen::VectorXf BuildObservation(const ReferenceState &reference);
  Eigen::VectorXf ProcessObservationTerm(size_t term_index,
                                         const Eigen::VectorXf &value);
  int GetObservationDim(const std::string &name) const;
  int ComputeObservationDim() const;
  void CalculateWarmupMotorCommand(const ReferenceState &reference);
  void CalculateMotorCommand(const Eigen::VectorXf &action);
  void SendMotorCommand();
  void AdvanceTimeStep();
  data::LinkInfo GetRobotAnchorState() const;

  std::shared_ptr<data::RlStampTrackingParam> param_;
  std::unique_ptr<TrackingPolicyModel> policy_;

  Eigen::VectorXi policy2deploy_joint_idx_;
  Eigen::VectorXd default_joint_q_;
  Eigen::VectorXd default_joint_pos_policy_;
  Eigen::VectorXd action_scale_policy_;
  Eigen::VectorXd joint_kp_;
  Eigen::VectorXd joint_kd_;

  Eigen::VectorXd q_real_;
  Eigen::VectorXd qd_real_;
  Eigen::VectorXd q_enter_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd qd_des_;
  Eigen::VectorXd tau_ff_des_;

  Eigen::VectorXf last_action_policy_;
  Eigen::VectorXf zero_observation_;
  std::vector<ObservationHistory> observation_histories_;
  Eigen::Matrix3f reference_alignment_rot_ = Eigen::Matrix3f::Identity();
  Eigen::Quaternionf reference_alignment_quat_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f reference_alignment_pos_ = Eigen::Vector3f::Zero();
  bool reference_alignment_initialized_ = false;
  bool policy_started_ = false;

  int time_step_ = 0;
  int iter_ = 0;
  bool finished_ = false;
};

} // namespace runner

REGISTER_RUNNER(RlStampTrackingRunner, "rl_stamp_tracking_runner", kMotion)
