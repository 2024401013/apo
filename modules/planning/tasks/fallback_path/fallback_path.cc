/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/tasks/fallback_path/fallback_path.h"
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

namespace {

struct PathBoundSummary {
  double first_lower = 0.0;
  double first_upper = 0.0;
  double first_width = 0.0;
  size_t min_width_index = 0;
  double min_width_s = 0.0;
  double min_width_lower = 0.0;
  double min_width_upper = 0.0;
  double min_width = std::numeric_limits<double>::infinity();
  bool has_invalid_bound = false;
  size_t first_invalid_index = 0;
  double first_invalid_s = 0.0;
  double first_invalid_lower = 0.0;
  double first_invalid_upper = 0.0;
  bool init_l_in_first_bound = false;
};

PathBoundSummary BuildPathBoundSummary(const PathBoundary& path_boundary,
                                       const double init_l) {
  PathBoundSummary summary;
  const auto& boundary = path_boundary.boundary();
  if (boundary.empty()) {
    return summary;
  }

  summary.first_lower = boundary.front().first;
  summary.first_upper = boundary.front().second;
  summary.first_width = summary.first_upper - summary.first_lower;
  summary.init_l_in_first_bound =
      init_l >= summary.first_lower && init_l <= summary.first_upper;

  for (size_t i = 0; i < boundary.size(); ++i) {
    const double lower = boundary[i].first;
    const double upper = boundary[i].second;
    const double width = upper - lower;
    const double s = static_cast<double>(i) * path_boundary.delta_s() +
                     path_boundary.start_s();
    if (width < summary.min_width) {
      summary.min_width = width;
      summary.min_width_index = i;
      summary.min_width_s = s;
      summary.min_width_lower = lower;
      summary.min_width_upper = upper;
    }
    if (lower > upper && !summary.has_invalid_bound) {
      summary.has_invalid_bound = true;
      summary.first_invalid_index = i;
      summary.first_invalid_s = s;
      summary.first_invalid_lower = lower;
      summary.first_invalid_upper = upper;
    }
  }
  return summary;
}

void LogPathOptimizerSummary(const std::string& event,
                             const PathBoundary& path_boundary,
                             const SLState& init_state,
                             const PathBoundSummary& summary) {
  ADEBUG << "[FALLBACK_PATH_OPT_DEBUG] " << event
         << ", label: " << path_boundary.label()
         << ", path_bound_size: " << path_boundary.boundary().size()
         << ", blocking_obstacle_id: " << path_boundary.blocking_obstacle_id()
         << ", init_l: " << init_state.second[0]
         << ", init_dl: " << init_state.second[1]
         << ", init_ddl: " << init_state.second[2]
         << ", first_lower_l: " << summary.first_lower
         << ", first_upper_l: " << summary.first_upper
         << ", first_width: " << summary.first_width
         << ", min_width_index: " << summary.min_width_index
         << ", min_width_s: " << summary.min_width_s
         << ", min_width_lower_l: " << summary.min_width_lower
         << ", min_width_upper_l: " << summary.min_width_upper
         << ", min_width: " << summary.min_width
         << ", has_invalid_bound: " << summary.has_invalid_bound
         << ", init_l_in_first_bound: " << summary.init_l_in_first_bound;
  if (summary.has_invalid_bound) {
    ADEBUG << "[FALLBACK_PATH_OPT_DEBUG] first invalid bound, label: "
           << path_boundary.label()
           << ", index: " << summary.first_invalid_index
           << ", s: " << summary.first_invalid_s
           << ", lower_l: " << summary.first_invalid_lower
           << ", upper_l: " << summary.first_invalid_upper;
  }
}

}  // namespace

bool FallbackPath::Init(const std::string& config_dir, const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<FallbackPathConfig>(&config_);
}

apollo::common::Status FallbackPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->IsChangeLanePath()) {
    return Status::OK();
  }
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status::OK();
  }
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status::OK();
  }
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    AERROR << "Path assessment failed";
  }

  return Status::OK();
}

bool FallbackPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  boundary->emplace_back();
  auto& path_bound = boundary->back();
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }
  // 2. Decide a rough boundary based on lane info and ADC's position
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }
  if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(
          *reference_line_info_, init_sl_state_, config_.extend_buffer(),
          &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on adc.";
    return false;
  }
  path_bound.set_label(absl::StrCat("fallback/", "self"));
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;
}

bool FallbackPath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  const auto& config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size,
                                     config.path_reference_l_weight());
    const PathBoundSummary path_bound_summary =
        BuildPathBoundSummary(path_boundary, init_sl_state_.second[0]);
    LogPathOptimizerSummary("before optimizer", path_boundary, init_sl_state_,
                            path_bound_summary);
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);
    LogPathOptimizerSummary(res_opt ? "after optimizer success"
                                    : "after optimizer failed",
                            path_boundary, init_sl_state_, path_bound_summary);
    if (res_opt) {
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data->push_back(std::move(path_data));
    }
  }
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

bool FallbackPath::AssessPath(std::vector<PathData>* candidate_path_data,
                              PathData* final_path) {
  PathData curr_path_data = candidate_path_data->back();
  RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
                  reference_line_info_);
  if (curr_path_data.Empty()) {
    ADEBUG << "Fallback Path: path data is empty.";
    return false;
  }
  // Check if the path is greatly off the reference line.
  if (PathAssessmentDeciderUtil::IsGreatlyOffReferenceLine(curr_path_data)) {
    ADEBUG << "Fallback Path: ADC is greatly off reference line.";
    return false;
  }
  // Check if the path is greatly off the road.
  if (PathAssessmentDeciderUtil::IsGreatlyOffRoad(*reference_line_info_,
                                                  curr_path_data)) {
    ADEBUG << "Fallback Path: ADC is greatly off road.";
    return false;
  }

  if (curr_path_data.Empty()) {
    AINFO << "Lane follow path is empty after trimed";
    return false;
  }
  *final_path = curr_path_data;
  return true;
}

}  // namespace planning
}  // namespace apollo
