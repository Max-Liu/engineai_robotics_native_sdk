#pragma once

#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"

namespace data {

class MotionStampParam : public BasicParam {
 public:
  MotionStampParam(std::string_view tag = "motion_stamp") : BasicParam(tag) { num_actions = joint_names.size(); }

  DEFINE_PARAM_SCOPE(scope_);

  std::string LOAD_PARAM(policy_file);
  std::string LOAD_PARAM(trajectory_file_npz);
  std::optional<std::vector<std::string>> LOAD_PARAM(trajectory_joint_names);

  std::vector<std::string> LOAD_PARAM(joint_names);
  Eigen::VectorXd LOAD_PARAM(joint_stiffness);
  Eigen::VectorXd LOAD_PARAM(joint_damping);
  Eigen::VectorXd LOAD_PARAM(default_joint_pos);

  std::vector<std::string> LOAD_PARAM(observation_names);
  std::vector<int> LOAD_PARAM(observation_history_lengths);
  Eigen::VectorXd LOAD_PARAM(action_scale);
  bool LOAD_PARAM(resident_control);
  std::optional<double> LOAD_PARAM(transition_duration_s);

  int num_actions;
};

}  // namespace data
