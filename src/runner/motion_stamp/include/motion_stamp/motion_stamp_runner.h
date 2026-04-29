#pragma once

#include <initializer_list>
#include <string_view>
#include <vector>

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "motion_stamp/motion_stamp_obs_registry.h"
#include "motion_stamp_param/motion_stamp_param.h"

#include "cnpy.h"
#include "math/mnn_model.h"
#include "parameter/global_config_initializer.h"

namespace runner {

class MotionStampRunner : public MotionRunner {
 public:
  MotionStampRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : MotionRunner(name, data_store) {
    param_ = data::ParamManager::create<data::MotionStampParam>();
  }
  ~MotionStampRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

 private:
  void CalculateObservation();
  void CalculateMotorCommand();
  void SendMotorCommand();
  bool RunEntryTransition();
  void initHistoryBuffers();
  void fillObsContextConstantPart();
  void updateFirstFrameYawAlignment();

  Eigen::MatrixXd LoadTrajectoryMatrix(std::initializer_list<std::string_view> candidate_keys, int row_index = 0);
  Eigen::MatrixXd npyFloatToMatrixXd(const cnpy::NpyArray& npy_array, int row_index = 0);
  Eigen::MatrixXd ReorderTrajectoryJointColumns(const Eigen::MatrixXd& matrix, std::string_view matrix_name) const;

  int GetObservationDim(const std::string& name) const;
  int ComputeTotalObservationDim() const;

  // --- Parameters and reference trajectory ---
  std::shared_ptr<data::MotionStampParam> param_;
  std::shared_ptr<const Eigen::MatrixXd> ref_joint_pos_all_;
  std::shared_ptr<const Eigen::MatrixXd> ref_joint_vel_all_;
  std::shared_ptr<const Eigen::MatrixXd> ref_body_pos_w_all_;
  std::shared_ptr<const Eigen::MatrixXd> ref_body_quat_w_all_;
  std::shared_ptr<const Eigen::MatrixXd> ref_body_lin_vel_w_all_;
  cnpy::npz_t trajectory_npz;
  int max_policy_step = 0;

  // --- Policy and observation ---
  std::unique_ptr<math::MNNModel> mlp_net_;
  Eigen::VectorXd mlp_net_observation_vec;
  std::shared_ptr<Eigen::VectorXd> mlp_net_action_;
  std::vector<Eigen::MatrixXd> observation_history_buffers_;

  motion_stamp_obs::ObsContext obs_ctx_;

  // --- First frame ---
  bool is_first_time_ = true;
  int policy_step = 0;
  bool log_first_policy_command_ = true;

  // --- Joint and mapping ---
  std::shared_ptr<Eigen::VectorXi> policy2deploy_joint_idx_;
  std::shared_ptr<Eigen::VectorXd> default_joint_q_;
  Eigen::VectorXd q_real_;
  Eigen::VectorXd qd_real_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd qd_des_;
  Eigen::VectorXd tau_ff_des_;
  Eigen::VectorXd joint_kp_;
  Eigen::VectorXd joint_kd_;
  Eigen::VectorXd action_scale_;
  Eigen::VectorXd transition_start_q_;
  Eigen::VectorXd transition_target_q_;
  int transition_iter_ = 0;
  double transition_duration_s_ = 0.0;
  bool entry_transition_active_ = false;

  Eigen::Matrix3d ref_init_yaw_rot_;
  Eigen::Matrix3d body_init_yaw_rot_;
};

}  // namespace runner

REGISTER_RUNNER(MotionStampRunner, "motion_stamp_runner", kMotion)
