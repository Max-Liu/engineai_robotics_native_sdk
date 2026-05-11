#pragma once

#include <string>
#include <vector>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"

namespace data {

class RlPowerShotTrackingParam : public BasicParam {
public:
  RlPowerShotTrackingParam(std::string_view tag = "rl_power_shot_tracking")
      : BasicParam(tag) {
    num_actions = static_cast<int>(joint_names.size());
  }

  DEFINE_PARAM_SCOPE(scope_);

  std::string LOAD_PARAM(policy_file);
  int LOAD_PARAM(time_step_total);

  std::vector<std::string> LOAD_PARAM(joint_names);
  Eigen::VectorXd LOAD_PARAM(joint_stiffness);
  Eigen::VectorXd LOAD_PARAM(joint_damping);
  Eigen::VectorXd LOAD_PARAM(default_joint_pos);
  Eigen::VectorXd LOAD_PARAM(action_scale);

  std::vector<std::string> LOAD_PARAM(observation_names);
  std::vector<int> LOAD_PARAM(observation_history_lengths);

  double LOAD_PARAM(transition_duration_s);
  bool LOAD_PARAM(loop_motion);
  bool LOAD_PARAM(reset_observation_history_on_loop);
  bool LOAD_PARAM(auto_transition);
  bool LOAD_PARAM(align_reference_to_robot_anchor);

  int num_actions = 0;
};

} // namespace data
