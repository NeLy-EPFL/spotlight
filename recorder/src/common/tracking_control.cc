#include "recorder/common/tracking_control.h"

#include "recorder/common/loop_monitors.h"

// Shared global variables and sysnchronization primitives
namespace {
std::mutex request_mutex;
std::condition_variable request_cond_var;
std::mutex response_mutex;
std::condition_variable response_cond_var;
std::queue<MotionStageRequest> request_queue;
std::map<int, MotionStageResponse> response_map;

std::ofstream
initialize_motion_stage_log_file(const std::string &save_directory) {
    std::filesystem::path filename =
        fs::path(save_directory) / "stage_position" / "stage_position.csv";

    // Remove the file if it exists (otherwise we'd be appending to it)
    if (std::filesystem::exists(filename)) {
        std::filesystem::remove(filename);
        spdlog::info(
            "Removed existing motion stage log file: {}", filename.string());
    }

    std::ofstream log_file((filename).string(), std::ios_base::app);
    if (!log_file.is_open()) {
        spdlog::error(
            "Failed to open motion stage log file: {}", filename.string());
    } else {
        spdlog::info("Opened motion stage log file: {}", filename.string());
    }
    log_file << "timestamp_us,x_pos_mm,y_pos_mm\n";
    return log_file;
}

double calculate_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

int image_binarize_threshold;

double software_x_min_mm = -std::numeric_limits<double>::infinity();
double software_x_max_mm = std::numeric_limits<double>::infinity();
double software_y_min_mm = -std::numeric_limits<double>::infinity();
double software_y_max_mm = std::numeric_limits<double>::infinity();
} // namespace

void motion_control_request_handler(
    const RecorderConfig &recorder_config,
    std::shared_ptr<TrackingControlState> tracking_control_state,
    std::shared_ptr<ProgramState> program_state) {
    MotionControl motion_control(recorder_config);

    // Query the stages' soft travel limits once at init so that we can
    // clamp target positions and avoid BADDATA rejections from the
    // controller.
    const double x_min_mm = motion_control.get_min_position(x_axis);
    const double x_max_mm = motion_control.get_max_position(x_axis);
    const double y_min_mm = motion_control.get_min_position(y_axis);
    const double y_max_mm = motion_control.get_max_position(y_axis);
    spdlog::info(
        "Motion stage travel limits: X=[{:.3f}, {:.3f}] mm, "
        "Y=[{:.3f}, {:.3f}] mm",
        x_min_mm,
        x_max_mm,
        y_min_mm,
        y_max_mm);

    tracking_control_state->motion_control_handler_ready.store(true);

    while (!program_state->to_quit.load()) {
        MotionStageRequest my_request;
        // Wait for a request
        {
            std::unique_lock<std::mutex> lock(request_mutex);
            request_cond_var.wait(lock, [program_state] {
                return !request_queue.empty() || program_state->to_quit.load();
            });

            // If there's still work to do, finish it even if told to stop
            if (!request_queue.empty()) {
                my_request = request_queue.front();
                request_queue.pop();
            } else {
                // Only way to reach here is if to_quit is true
                assert(program_state->to_quit.load());
                spdlog::info("Motion stage request handler thread "
                             "is breaking out of loop.");
                break;
            }
        }

        MotionStageResponse my_response;
        if (my_request.request_type == get_current_position) {
            MotionStagePosition current_position = {
                motion_control.get_position(x_axis),
                motion_control.get_position(y_axis),
                absolute};
            my_response.position = current_position;
        } else if (my_request.request_type == set_target_position) {
            bool wait_for_completion = false;
            if (my_request.position.position_type == absolute) {
                double target_x = std::clamp(
                    my_request.position.x_pos_mm, x_min_mm, x_max_mm);
                double target_y = std::clamp(
                    my_request.position.y_pos_mm, y_min_mm, y_max_mm);
                if (target_x != my_request.position.x_pos_mm ||
                    target_y != my_request.position.y_pos_mm) {
                    spdlog::warn(
                        "Target stage position ({:.3f}, {:.3f}) mm clamped "
                        "to ({:.3f}, {:.3f}) mm to stay within travel "
                        "limits X=[{:.3f}, {:.3f}], Y=[{:.3f}, {:.3f}].",
                        my_request.position.x_pos_mm,
                        my_request.position.y_pos_mm,
                        target_x,
                        target_y,
                        x_min_mm,
                        x_max_mm,
                        y_min_mm,
                        y_max_mm);
                }
                motion_control.move_absolute(
                    x_axis, target_x, wait_for_completion, my_request.velocity);
                motion_control.move_absolute(
                    y_axis, target_y, wait_for_completion, my_request.velocity);
            } else {
                // Convert the relative request to an absolute target so we
                // can clamp against the travel limits before issuing the
                // move.
                double current_x = motion_control.get_position(x_axis);
                double current_y = motion_control.get_position(y_axis);
                double target_x = std::clamp(
                    current_x + my_request.position.x_pos_mm,
                    x_min_mm,
                    x_max_mm);
                double target_y = std::clamp(
                    current_y + my_request.position.y_pos_mm,
                    y_min_mm,
                    y_max_mm);
                motion_control.move_absolute(
                    x_axis, target_x, wait_for_completion, my_request.velocity);
                motion_control.move_absolute(
                    y_axis, target_y, wait_for_completion, my_request.velocity);
            }
            my_response.set_success = true;
        } else if (my_request.request_type == wait_until_idle) {
            motion_control.wait_until_idle(x_axis);
            motion_control.wait_until_idle(y_axis);
            my_response.is_idle = true;
        } else if (my_request.request_type == check_if_idle) {
            my_response.is_idle = motion_control.check_if_idle(x_axis) &&
                                  motion_control.check_if_idle(y_axis);
        } else if (my_request.request_type == start_homing) {
            bool wait_for_completion = false;
            motion_control.home(x_axis, wait_for_completion);
            motion_control.home(y_axis, wait_for_completion);
            my_response.set_success = true;
        } else {
            spdlog::critical(
                "Motion stage request handler thread received unknown "
                "request type: {}",
                static_cast<int>(my_request.request_type));
            throw std::runtime_error(
                "Motion stage request handler thread received unknown "
                "request type.");
        }

        // Send response back
        {
            std::lock_guard<std::mutex> lock(response_mutex);
            response_map[my_request.client_id_hash] = my_response;
        }
        response_cond_var.notify_all();
    }
    spdlog::info("Motion stage request handler thread stopped.");
}

// Run one automatic-tracking update: grab the latest behavior image, locate
// the fly, and move the stage to re-center it if it has drifted far enough.
void update_tracking_target(
    const RecorderConfig &recorder_config,
    ActiveAreaMask &active_area_mask,
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
    std::shared_ptr<TrackingControlState> tracking_control_state,
    const CalibrationParams &behavior_cam_calibration_params,
    float tracking_distance_threshold_mm,
    float default_velocity) {
    cv::Mat my_behavior_image =
        behavior_recording_state->latest_frame_holder->get_latest_frame_data()
            .image;
    reorient_behavior_image(my_behavior_image, my_behavior_image);
    cv::Mat active_area_mask_curr_view = active_area_mask.warp_to_current_view(
        my_behavior_image, get_current_motion_stage_position());

    MotionStagePosition my_motion_stage_position;
    {
        std::lock_guard<std::mutex> lock(
            tracking_control_state->latest_motion_stage_position_mutex);
        my_motion_stage_position =
            tracking_control_state->latest_motion_stage_position;
    }

    bool is_found = false;
    double physical_pos_x = 0;
    double physical_pos_y = 0;
    if (behavior_recording_state->behavior_camera &&
        behavior_recording_state->behavior_camera->is_ready()) {
        std::tie(is_found, physical_pos_x, physical_pos_y) =
            calculate_fly_position_absolute_mm(
                my_behavior_image,
                my_motion_stage_position,
                active_area_mask_curr_view,
                behavior_cam_calibration_params,
                recorder_config);
    }

    if (is_found) {
        auto [current_physical_pos_x, current_physical_pos_y] =
            behavior_cam_calibration_params
                .stage_pos_and_pixel_pos_to_physical_pos(
                    my_motion_stage_position.x_pos_mm,
                    my_motion_stage_position.y_pos_mm,
                    my_behavior_image.rows / 2,
                    my_behavior_image.cols / 2);

        double distance_to_target = calculate_distance(
            physical_pos_x,
            physical_pos_y,
            current_physical_pos_x,
            current_physical_pos_y);

        // Only move the stage once the fly has drifted far enough from
        // the center of the view. Holding still when it is already close
        // avoids jittering, reduces wear on the motors, and reduces
        // mechanical resonance. (Don't `continue` here: that would skip
        // the end-of-cycle sleep below and busy-loop the thread, which
        // also floods the motion-control request queue.)
        if (distance_to_target >= tracking_distance_threshold_mm) {
            // X stage should move in the OPPOSITE direction: the arena
            // is facing downward, so the +x direction of the arena is
            // the opposite of the +x direction of the stage.
            double dx = -1 * (physical_pos_x - current_physical_pos_x);
            double dy = physical_pos_y - current_physical_pos_y;
            MotionStagePosition target_motion_stage_position = {
                my_motion_stage_position.x_pos_mm + dx,
                my_motion_stage_position.y_pos_mm + dy,
                absolute};
            set_target_motion_stage_position(
                target_motion_stage_position, default_velocity);
        }
    }
}

void tracking_controller(
    const RecorderConfig &recorder_config,
    ActiveAreaMask &active_area_mask,
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
    std::shared_ptr<TrackingControlState> tracking_control_state,
    const CalibrationParams &behavior_cam_calibration_params,
    std::shared_ptr<ProgramState> program_state) {
    size_t retry_count = 0;
    while (!tracking_control_state->motion_control_handler_ready.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retry_count++;
        if (retry_count % 10 == 0) {
            spdlog::warn("Motion control handler is not ready.");
        }
    }

    const int tracking_update_frequency =
        recorder_config.get_parameter<int>("tracking", "update_frequency_hz");
    LoopRateLimiter rate_limiter(
        "Tracking controller thread", tracking_update_frequency);

    const float tracking_distance_threshold_mm =
        recorder_config.get_parameter<float>(
            "tracking", "distance_threshold_for_moving_mm");

    const float default_velocity = recorder_config.get_parameter<float>(
        "motion_control", "default_velocity_mm_per_s");

    image_binarize_threshold = recorder_config.get_parameter<int>(
        "tracking", "image_binarize_threshold");

    while (!program_state->to_quit.load()) {
        rate_limiter.start_cycle();

        if (!tracking_control_state->tracking_on.load()) {
            // Nothing to do here
        } else if (!tracking_control_state->should_override_tracking.load()) {
            update_tracking_target(
                recorder_config,
                active_area_mask,
                behavior_recording_state,
                tracking_control_state,
                behavior_cam_calibration_params,
                tracking_distance_threshold_mm,
                default_velocity);
        } else {
            // spdlog::debug("Tracking controller is overriding tracking.");
            MotionStagePosition current_pos =
                get_current_motion_stage_position();
            double distance_to_target = calculate_distance(
                tracking_control_state->overriding_pos_x.load(),
                tracking_control_state->overriding_pos_y.load(),
                current_pos.x_pos_mm,
                current_pos.y_pos_mm);

            if (distance_to_target < tracking_distance_threshold_mm &&
                check_if_motion_stage_idle()) {
                // Reached the click-to-move target; hand control back to
                // automatic tracking. (Don't `continue`: fall through to the
                // end-of-cycle sleep below so the thread doesn't busy-loop.)
                tracking_control_state->should_override_tracking.store(false);
            } else {
                MotionStagePosition target_pos = {
                    tracking_control_state->overriding_pos_x.load(),
                    tracking_control_state->overriding_pos_y.load(),
                    absolute};
                set_target_motion_stage_position(target_pos, default_velocity);
            }
        }
        rate_limiter.sleep_until_next_cycle();
    }
}

ActiveAreaMask::ActiveAreaMask(
    const std::string &arena_spec_dir,
    double boundary_margin_mm,
    LinearMapper2x2to2 &stage_and_pixel_to_physical)
    : stage_and_pixel_to_physical(stage_and_pixel_to_physical) {
    // Load arena metadata
    fs::path metadata_path = fs::path(arena_spec_dir) / "metadata.yaml";
    YAML::Node metadata = YAML::LoadFile(metadata_path.string());
    if (!metadata["unit"] || metadata["unit"].as<std::string>() != "mm") {
        throw std::runtime_error(
            "Arena metadata unit is not 'mm': " + metadata_path.string());
    }
    auto arena_dim = metadata["arena_dim"].as<std::vector<double>>();
    arena_width_mm = arena_dim[0];
    arena_height_mm = arena_dim[1];
    resolution_mm_per_pixel =
        metadata["active_area_raster_resolution"].as<double>();

    // Load rasterized active area mask
    fs::path mask_path = fs::path(arena_spec_dir) / "active_area.png";
    full_arena_mask = cv::imread(mask_path.string(), cv::IMREAD_GRAYSCALE);
    if (full_arena_mask.empty()) {
        throw std::runtime_error(
            "Failed to load active area mask from: " + mask_path.string());
    }
    int expect_cols =
        static_cast<int>(arena_width_mm / resolution_mm_per_pixel);
    int expect_rows =
        static_cast<int>(arena_height_mm / resolution_mm_per_pixel);
    if (full_arena_mask.cols != expect_cols ||
        full_arena_mask.rows != expect_rows) {
        throw std::runtime_error(
            "Active area mask has incorrect dimensions: " + mask_path.string());
    }

    // Shrink active area mask by boundary margin
    int boundary_margin_pixels =
        static_cast<int>(boundary_margin_mm / resolution_mm_per_pixel);
    cv::Mat erosion_kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(
            2 * boundary_margin_pixels + 1, 2 * boundary_margin_pixels + 1),
        cv::Point(boundary_margin_pixels, boundary_margin_pixels));
    cv::erode(
        full_arena_mask,
        full_arena_mask,
        erosion_kernel,
        cv::Point(-1, -1), // anchor (default)
        1,                 // iterations (default)
        // Set border value to 0 so that the erosion treats arena walls as
        // out-of-arena boundaries, so the edges are shrunk even if there
        // is no black pixels along the edges.
        cv::BORDER_CONSTANT, // borderType
        cv::Scalar(0));      // borderValue

    // Build the affine matrix that maps a camera pixel (col, row) to the
    // corresponding arena-mask pixel (col, row) when stage is at zero.
    // With WARP_INVERSE_MAP, warpAffine uses this matrix as:
    //   mask_col = M[0,0]*cam_col + M[0,1]*cam_row + M[0,2]
    //   mask_row = M[1,0]*cam_col + M[1,1]*cam_row + M[1,2]
    // which is exactly (stage_and_pixel_to_physical(stage=0, pixel) / R).
    transform_matrix_at_zero_stage_pos_ =
        (cv::Mat_<double>(2, 3) << stage_and_pixel_to_physical.w_x2_to_x,
         stage_and_pixel_to_physical.w_y2_to_x,
         stage_and_pixel_to_physical.bias_x,
         stage_and_pixel_to_physical.w_x2_to_y,
         stage_and_pixel_to_physical.w_y2_to_y,
         stage_and_pixel_to_physical.bias_y);
    transform_matrix_at_zero_stage_pos_ /= resolution_mm_per_pixel;
}

cv::Mat ActiveAreaMask::warp_to_current_view(
    const cv::Mat &current_image, MotionStagePosition stage_pos) const {
    // Add contribution of non-zero stage position to the transformation matrix
    double x_offset =
        (stage_and_pixel_to_physical.w_x1_to_x * stage_pos.x_pos_mm +
         stage_and_pixel_to_physical.w_y1_to_x * stage_pos.y_pos_mm);
    double y_offset =
        (stage_and_pixel_to_physical.w_x1_to_y * stage_pos.x_pos_mm +
         stage_and_pixel_to_physical.w_y1_to_y * stage_pos.y_pos_mm);
    cv::Mat transform_matrix = transform_matrix_at_zero_stage_pos_.clone();
    transform_matrix.at<double>(0, 2) += x_offset / resolution_mm_per_pixel;
    transform_matrix.at<double>(1, 2) += y_offset / resolution_mm_per_pixel;

    // Apply affine transform
    cv::Mat warped_mask;
    cv::warpAffine(
        full_arena_mask,
        warped_mask,
        transform_matrix,
        current_image.size(),
        cv::WARP_INVERSE_MAP | cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));
    return warped_mask;
}

void motion_stage_position_logger(
    const RecorderConfig &recorder_config,
    std::shared_ptr<TrackingControlState> tracking_control_state,
    std::shared_ptr<SaveDirectory> save_directory,
    std::shared_ptr<ProgramState> program_state) {
    const int position_logging_freq = recorder_config.get_parameter<int>(
        "motion_control", "position_logging_frequency_hz");
    LoopRateLimiter rate_limiter(
        "Motion stage position logging thread", position_logging_freq);

    std::ofstream log_file;
    bool was_recording_last_iter = false;

    while (!program_state->to_quit.load()) {
        rate_limiter.start_cycle();

        // Get current position
        uint64_t start_time = get_current_time_microseconds();
        MotionStagePosition current_position =
            get_current_motion_stage_position();

        // Update latest position for other threads
        {
            std::lock_guard<std::mutex> lock(
                tracking_control_state->latest_motion_stage_position_mutex);
            tracking_control_state->latest_motion_stage_position =
                current_position;
        }

        // Log position
        if (program_state->is_recording.load()) {
            if (!was_recording_last_iter) {
                // This is the start of a new recording. We need to initalize
                // the log file.
                log_file = initialize_motion_stage_log_file(
                    save_directory->get_directory());
                spdlog::info(
                    "Stage position log file initialized under {}. "
                    "Stage position logging starts now.",
                    save_directory->get_directory().c_str());
            }

            log_file << start_time << "," << current_position.x_pos_mm << ","
                     << current_position.y_pos_mm << "\n";
            log_file.flush();

            was_recording_last_iter = true;
        } else {
            if (was_recording_last_iter) {
                assert(log_file.is_open());
                spdlog::info(
                    "Stage position logging stopped. Closing log file.");
                log_file.close();
            }
            assert(!log_file.is_open());
            was_recording_last_iter = false;
        }

        rate_limiter.sleep_until_next_cycle();
    }
    spdlog::info("Motion stage position logging thread stopped.");
}

std::tuple<bool, double, double> calculate_fly_position_absolute_mm(
    const cv::Mat &behavior_image,
    MotionStagePosition stage_position,
    const cv::Mat &active_area_mask_curr_view,
    const CalibrationParams &behavior_cam_calibration_params,
    const RecorderConfig &recorder_config) {
    bool is_found = false;
    double physical_pos_x_mm = 0;
    double physical_pos_y_mm = 0;

    if (behavior_image.empty()) {
        spdlog::warn(
            "Input behavior image is empty. It's normal if this happens "
            "only one or two times at the start of recording.");
        return {is_found, physical_pos_x_mm, physical_pos_y_mm};
    }
    if (!behavior_cam_calibration_params.is_defined) {
        // Cannot map pixel positions to physical positions because the
        // calibration model has not been defined yet
        return {is_found, physical_pos_x_mm, physical_pos_y_mm};
    }

    // Zero out pixels that fall outside the active arena area.
    cv::Mat blacked_out_image =
        cv::Mat::zeros(behavior_image.size(), behavior_image.type());
    behavior_image.copyTo(blacked_out_image, active_area_mask_curr_view);

    assert(blacked_out_image.channels() == 1);

    // Threshold the image at a cutout of 100
    // spdlog::debug("Thresholding");
    cv::Mat binary_image;
    cv::threshold(
        blacked_out_image,
        binary_image,
        image_binarize_threshold,
        255,
        cv::THRESH_BINARY);
    if (binary_image.empty()) {
        spdlog::error("Binary image after thresholding is empty");
        return {is_found, physical_pos_x_mm, physical_pos_y_mm};
    }
    // display binary_image for debugging
    // cv::imshow("binary_image", binary_image);
    if (cv::countNonZero(binary_image) == 0) {
        return {is_found, physical_pos_x_mm, physical_pos_y_mm};
    }

    // Apply morphological opening and closing with a smaller kernel
    // spdlog::debug("Getting kernel for morphological operations");
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // spdlog::debug("Applying morphological opening");
    cv::Mat opened_image;
    cv::morphologyEx(binary_image, opened_image, cv::MORPH_OPEN, kernel);

    // spdlog::debug("Applying morphological closing");
    cv::Mat morphed_image;
    cv::morphologyEx(opened_image, morphed_image, cv::MORPH_CLOSE, kernel);

    // spdlog::debug("Finding connected components");
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(
        morphed_image, labels, stats, centroids);

    // Find the largest connected component (excluding the background which is
    // label 0).
    // spdlog::debug("Finding the largest connected component");
    int max_area = 0;
    int max_label = 0;
    for (int i = 1; i < num_labels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > max_area) {
            max_area = area;
            max_label = i;
        }
    }

    // If the largest connected component is large enough, this is the fly

    int min_fly_size_sq_pixels = recorder_config.get_parameter<int>(
        "tracking", "min_fly_size_sq_pixels");
    if (max_label > 0 && max_area > min_fly_size_sq_pixels) {
        // spdlog::debug(
        //     "Getting the center of mass of the largest connected component");
        double center_of_mass_col = centroids.at<double>(max_label, 0); // x
        double center_of_mass_row = centroids.at<double>(max_label, 1); // y

        auto [x, y] = behavior_cam_calibration_params
                          .stage_pos_and_pixel_pos_to_physical_pos(
                              stage_position.x_pos_mm,
                              stage_position.y_pos_mm,
                              center_of_mass_row,
                              center_of_mass_col);
        is_found = true;
        physical_pos_x_mm = x;
        physical_pos_y_mm = y;

        // spdlog::debug(
        //     "Fly found at pixel ({:.2f}, {:.2f}), physical ({:.2f}, {:.2f})",
        //     centerOfMassCol, centerOfMassRow, physicalPosXMm, physicalPosYMm
        // );
    }

    return std::make_tuple(is_found, physical_pos_x_mm, physical_pos_y_mm);
}

MotionStagePosition get_current_motion_stage_position() {
    size_t my_thread_id_hash = get_my_thread_id_hash();

    // Push request
    MotionStageRequest my_request;
    my_request.client_id_hash = my_thread_id_hash;
    my_request.request_type = get_current_position;
    {
        std::lock_guard<std::mutex> lock(request_mutex);
        request_queue.push(my_request);
    }
    request_cond_var.notify_one();

    // Wait for response
    MotionStageResponse my_response;
    {
        std::unique_lock<std::mutex> lock(response_mutex);
        response_cond_var.wait(lock, [my_thread_id_hash] {
            return response_map.find(my_thread_id_hash) != response_map.end();
        });
        my_response = response_map[my_thread_id_hash];
        response_map.erase(my_thread_id_hash);
    }
    return my_response.position;
}

void set_target_motion_stage_position(
    MotionStagePosition target_position, float velocity) {
    if (target_position.position_type == absolute) {
        double clamped_x = std::clamp(
            target_position.x_pos_mm, software_x_min_mm, software_x_max_mm);
        double clamped_y = std::clamp(
            target_position.y_pos_mm, software_y_min_mm, software_y_max_mm);
        if (clamped_x != target_position.x_pos_mm ||
            clamped_y != target_position.y_pos_mm) {
            spdlog::warn(
                "Target stage position ({:.3f}, {:.3f}) mm clamped to "
                "({:.3f}, {:.3f}) mm by software motion stage limits "
                "X=[{:.3f}, {:.3f}], Y=[{:.3f}, {:.3f}].",
                target_position.x_pos_mm,
                target_position.y_pos_mm,
                clamped_x,
                clamped_y,
                software_x_min_mm,
                software_x_max_mm,
                software_y_min_mm,
                software_y_max_mm);
        }
        target_position.x_pos_mm = clamped_x;
        target_position.y_pos_mm = clamped_y;
    }

    size_t my_thread_id_hash = get_my_thread_id_hash();

    // Push request
    MotionStageRequest my_request;
    my_request.client_id_hash = my_thread_id_hash;
    my_request.request_type = set_target_position;
    my_request.position = target_position;
    my_request.velocity = velocity;
    {
        std::lock_guard<std::mutex> lock(request_mutex);
        request_queue.push(my_request);
    }
    request_cond_var.notify_one();

    // Wait for response
    MotionStageResponse my_response;
    {
        std::unique_lock<std::mutex> lock(response_mutex);
        response_cond_var.wait(lock, [my_thread_id_hash] {
            return response_map.find(my_thread_id_hash) != response_map.end();
        });
        my_response = response_map[my_thread_id_hash];
        response_map.erase(my_thread_id_hash);
    }
    if (!my_response.set_success) {
        spdlog::critical("Failed to set target motion stage position.");
        throw std::runtime_error("Failed to set target motion stage position.");
    }
}

void wait_until_motion_stage_idle_sync() {
    size_t my_thread_id_hash = get_my_thread_id_hash();

    // Push request
    MotionStageRequest my_request;
    my_request.client_id_hash = my_thread_id_hash;
    my_request.request_type = wait_until_idle;
    {
        std::lock_guard<std::mutex> lock(request_mutex);
        request_queue.push(my_request);
    }
    request_cond_var.notify_one();

    // Wait for response
    MotionStageResponse my_response;
    {
        std::unique_lock<std::mutex> lock(response_mutex);
        response_cond_var.wait(lock, [my_thread_id_hash] {
            return response_map.find(my_thread_id_hash) != response_map.end();
        });
        my_response = response_map[my_thread_id_hash];
        response_map.erase(my_thread_id_hash);
    }
    if (!my_response.is_idle) {
        spdlog::critical(
            "Motion stage request handler thread responed to WAIT_UNTIL_IDLE "
            "request, but the stages are not idle.");
        throw std::runtime_error(
            "Motion stage request handler thread responed to WAIT_UNTIL_IDLE "
            "request, but the stages are not idle.");
    }
}

/**
 * @brief Waits asynchronously until the motion stage becomes idle.
 *
 * This function checks if the motion stage is idle by calling
 * `checkIfMotionStageIdle` every 500 ms. Like waitUntilMotionStageIdleSync(),
 * this function is blocking, but it doesn't block the thread that handles
 * requests to read form / write to the hardware, so other threads who need to
 * interface with the motion stages can still do it.
 */
void wait_until_motion_stage_idle_async() {
    while (!check_if_motion_stage_idle()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

bool check_if_motion_stage_idle() {
    size_t my_thread_id_hash = get_my_thread_id_hash();

    // Push request
    MotionStageRequest my_request;
    my_request.client_id_hash = my_thread_id_hash;
    my_request.request_type = check_if_idle;
    {
        std::lock_guard<std::mutex> lock(request_mutex);
        request_queue.push(my_request);
    }
    request_cond_var.notify_one();

    // Wait for response
    MotionStageResponse my_response;
    {
        std::unique_lock<std::mutex> lock(response_mutex);
        response_cond_var.wait(lock, [my_thread_id_hash] {
            return response_map.find(my_thread_id_hash) != response_map.end();
        });
        my_response = response_map[my_thread_id_hash];
        response_map.erase(my_thread_id_hash);
    }
    return my_response.is_idle;
}

void start_homing_motion_stage() {
    size_t my_thread_id_hash = get_my_thread_id_hash();

    // Push request
    MotionStageRequest my_request;
    my_request.client_id_hash = my_thread_id_hash;
    my_request.request_type = start_homing;
    {
        std::lock_guard<std::mutex> lock(request_mutex);
        request_queue.push(my_request);
    }
    request_cond_var.notify_one();

    // Wait for response
    MotionStageResponse my_response;
    {
        std::unique_lock<std::mutex> lock(response_mutex);
        response_cond_var.wait(lock, [my_thread_id_hash] {
            return response_map.find(my_thread_id_hash) != response_map.end();
        });
        my_response = response_map[my_thread_id_hash];
        response_map.erase(my_thread_id_hash);
    }
    if (!my_response.set_success) {
        spdlog::critical("Failed to start homing motion stage.");
        throw std::runtime_error("Failed to start homing motion stage.");
    }
}

void set_motion_stage_limits(
    double x_min_mm, double x_max_mm, double y_min_mm, double y_max_mm) {
    software_x_min_mm = x_min_mm;
    software_x_max_mm = x_max_mm;
    software_y_min_mm = y_min_mm;
    software_y_max_mm = y_max_mm;
    spdlog::info(
        "Software motion stage limits set: X=[{:.3f}, {:.3f}], "
        "Y=[{:.3f}, {:.3f}] mm",
        x_min_mm,
        x_max_mm,
        y_min_mm,
        y_max_mm);
}

void stop_motion_control_request_handler(
    std::shared_ptr<ProgramState> program_state) {
    if (!program_state->to_quit.load()) {
        spdlog::critical(
            "stop_motion_control_request_handler() called but to_quit "
            "is not set to true. This shouldn't happen.");
        throw std::runtime_error(
            "stop_motion_control_request_handler() called but to_quit "
            "is not set to true. This shouldn't happen.");
    } else {
        request_cond_var.notify_one();
    }
}