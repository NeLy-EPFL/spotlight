#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <set>
#include <thread>
#include <tuple>

#include <spdlog/spdlog.h>

#include "recorder/common/behavior_recording.h"
#include "recorder/common/calibration.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/motion_control.h"

class ActiveAreaMask {
  public:
    cv::Mat full_arena_mask;
    double resolution_mm_per_pixel;
    double arena_width_mm;
    double arena_height_mm;
    LinearMapper2x2to2 &stage_and_pixel_to_physical;

    ActiveAreaMask(
        const std::string &arena_spec_dir,
        double boundary_margin_mm,
        LinearMapper2x2to2 &stage_and_pixel_to_physical);
    cv::Mat warp_to_current_view(
        const cv::Mat &current_image, MotionStagePosition stage_pos) const;

  private:
    cv::Mat transform_matrix_at_zero_stage_pos_;
};

struct TrackingControlState {
    std::atomic<bool> motion_control_handler_ready = false;
    MotionStagePosition latest_motion_stage_position;
    std::mutex latest_motion_stage_position_mutex;
    std::atomic<bool> tracking_on = true;
    std::atomic<bool> should_override_tracking = false;
    std::atomic<double> overriding_pos_x; // in mm
    std::atomic<double> overriding_pos_y; // in mm
};

// Hardware controller thread
void motion_control_request_handler(
    const RecorderConfig &recorder_config,
    const std::shared_ptr<TrackingControlState>& tracking_control_state,
    const std::shared_ptr<ProgramState>& program_state);

// Tracking thread
void tracking_controller(
    const RecorderConfig &recorder_config,
    ActiveAreaMask &active_area_mask,
    const std::shared_ptr<BehaviorRecordingState>& behavior_recording_state,
    const std::shared_ptr<TrackingControlState>& tracking_control_state,
    const CalibrationParams &behavior_cam_calibration_params,
    const std::shared_ptr<ProgramState>& program_state);

// Position logging thread
void motion_stage_position_logger(
    const RecorderConfig &recorder_config,
    const std::shared_ptr<TrackingControlState>& tracking_control_state,
    const std::shared_ptr<SaveDirectory>& save_directory,
    const std::shared_ptr<ProgramState>& program_state);

// Global API functions
// Aside from get_current_motion_stage_position(), they are all async.
MotionStagePosition get_current_motion_stage_position();
void set_target_motion_stage_position(
    MotionStagePosition target_position, float velocity);
void set_motion_stage_limits(
    double x_min_mm, double x_max_mm, double y_min_mm, double y_max_mm);
void wait_until_motion_stage_idle_sync();
void wait_until_motion_stage_idle_async();
bool check_if_motion_stage_idle();
void start_homing_motion_stage();
void stop_motion_control_request_handler(
    const std::shared_ptr<ProgramState>& program_state);

// High-level helper functions
std::tuple<bool, double, double> calculate_fly_position_absolute_mm(
    const cv::Mat &behavior_image,
    MotionStagePosition stage_position,
    const cv::Mat &active_area_mask_curr_view,
    const CalibrationParams &behavior_cam_calibration_params,
    const RecorderConfig &recorder_config);
