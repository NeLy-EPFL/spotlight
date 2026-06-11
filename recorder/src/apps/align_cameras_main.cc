#include "recorder/apps/align_cameras.h"

#define DISPLAY_DOWNSAMPLE_FACTOR 3

namespace {

int full_muscle_image_width;
int full_muscle_image_height;
int muscle_image_roi_width;
int muscle_image_roi_height;
int user_selected_center_x_display = -1;
int user_selected_center_y_display = -1;
std::filesystem::path profile_dir;
std::unique_ptr<ArduinoCommunication> arduino_communication = nullptr;

std::tuple<int, int>
display_to_camera_sensor_coords(int display_x, int display_y) {
    // Reverse the 90 degrees counterclockwise rotation and scale back up
    int sensor_x =
        full_muscle_image_width - (display_y * DISPLAY_DOWNSAMPLE_FACTOR) - 1;
    int sensor_y = display_x * DISPLAY_DOWNSAMPLE_FACTOR;
    return std::make_tuple(sensor_x, sensor_y);
}

std::tuple<int, int>
camera_sensor_to_display_coords(int sensor_x, int sensor_y) {
    // This is the opposite of display_to_camera_sensor_coords
    int display_x = sensor_y / DISPLAY_DOWNSAMPLE_FACTOR;
    int display_y =
        (full_muscle_image_width - sensor_x - 1) / DISPLAY_DOWNSAMPLE_FACTOR;
    return std::make_tuple(display_x, display_y);
}

MuscleCameraROI
get_roi_from_display_center(int x_center_display, int y_center_display) {
    auto [muscle_camera_center_x_sensor, muscle_camera_center_y_sensor] =
        display_to_camera_sensor_coords(x_center_display, y_center_display);
    int x_offset = muscle_camera_center_x_sensor - (muscle_image_roi_width / 2);
    int y_offset =
        muscle_camera_center_y_sensor - (muscle_image_roi_height / 2);
    // PCO ROI quantization: x offsets must be multiples of 32, y offsets
    // multiples of 8.
    int x0 = round_to_nearest_valid_muscle_cam_horizontal(x_offset) + 1;
    int x1 = (x0 - 1) + muscle_image_roi_width;
    int y0 = round_to_nearest_valid_muscle_cam_vertical(y_offset) + 1;
    int y1 = (y0 - 1) + muscle_image_roi_height;

    MuscleCameraROI roi(x0, x1, y0, y1);
    return roi;
}

void draw_muscle_image_roi(cv::Mat &image) {
    // Add red dot to muscle camera image
    if (user_selected_center_x_display == -1 ||
        user_selected_center_y_display == -1) {
        // User didn't select a center point yet
        return;
    }

    // Draw user-selected center point
    cv::Scalar red_color(0, 0, 255);
    cv::Point point_ideal(
        user_selected_center_x_display, user_selected_center_y_display);
    cv::circle(image, point_ideal, 5, red_color, -1);

    // Figure out center point coords on the camera sensor
    MuscleCameraROI roi = get_roi_from_display_center(
        user_selected_center_x_display, user_selected_center_y_display);

    // Draw closest feasible center point
    auto [x_center_sensor_actual, y_center_sensor_actual] = roi.get_center_xy();
    auto [x_center_display_actual, y_center_display_actual] =
        camera_sensor_to_display_coords(
            x_center_sensor_actual, y_center_sensor_actual);
    cv::Scalar blue_color(255, 0, 0);
    cv::Point point_actual(x_center_display_actual, y_center_display_actual);
    cv::circle(image, point_actual, 3, blue_color, -1);

    // Draw ROI rectangle
    auto [display_x0, display_y0] =
        camera_sensor_to_display_coords(roi.x0, roi.y0);
    auto [display_x1, display_y1] =
        camera_sensor_to_display_coords(roi.x1, roi.y1);
    cv::rectangle(
        image,
        cv::Point(display_x0, display_y0),
        cv::Point(display_x1, display_y1),
        blue_color,
        2); // thickness
}

void on_mouse(int event, int x, int y, int flags, void *userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        user_selected_center_x_display = x;
        user_selected_center_y_display = y;

        // Convert display coordinates to camera sensor coordinates
        auto [sensor_x, sensor_y] = display_to_camera_sensor_coords(x, y);
        spdlog::info(
            "Muscle camera center point set at (x={}, y={}) on "
            "the displayed image, which translates to (x={}, y={}) "
            "on the camera sensor.",
            x,
            y,
            sensor_x,
            sensor_y);
    }
}

void add_cross_to_behavior_image(cv::Mat &image) {
    // Add a cross to the middle of the behavior image to help with alignment
    cv::line(
        image,
        cv::Point(image.cols / 2, 0),
        cv::Point(image.cols / 2, image.rows),
        cv::Scalar(255, 255, 255),
        1);
    cv::line(
        image,
        cv::Point(0, image.rows / 2),
        cv::Point(image.cols, image.rows / 2),
        cv::Scalar(255, 255, 255),
        1);
}

bool try_save_selected_roi(const std::filesystem::path &profile_dir) {
    // Validate the user-selected ROI center and, if it is in bounds, write the
    // resulting ROI to <profile_dir>/muscle_camera_roi.yaml. Returns true when
    // an ROI was successfully saved (caller should break out of the streaming
    // loop), false otherwise (caller should continue).
    if (user_selected_center_x_display == -1 ||
        user_selected_center_y_display == -1) {
        spdlog::error("ROI center not set yet. Please select a center point on "
                      "the muscle camera image before saving the ROI.");
        return false;
    }

    spdlog::info(
        "User selected muscle camera center point at (x={}, y={})",
        user_selected_center_x_display,
        user_selected_center_y_display);
    MuscleCameraROI roi = get_roi_from_display_center(
        user_selected_center_x_display, user_selected_center_y_display);
    if (roi.x0 <= 0 || roi.y0 <= 0 || roi.x1 > full_muscle_image_width ||
        roi.y1 > full_muscle_image_height) {
        spdlog::error("Selected ROI out of bound. Please try again.");
        return false;
    }

    std::filesystem::path roi_file_path =
        profile_dir / "muscle_camera_roi.yaml";
    spdlog::info(
        "Saving muscle camera ROI (x0={}, x1={}, y0={}, y1={}) to {}",
        roi.x0,
        roi.x1,
        roi.y0,
        roi.y1,
        roi_file_path.string());
    roi.to_file(roi_file_path);

    return true;
}

void setup_display_windows(const RecorderConfig &recorder_config) {
    // Figure out window display size (note width/height are swapped because
    // images are roatated)
    int behavior_image_display_width =
        recorder_config.get_parameter<int>("behavior_camera", "roi_height") /
        DISPLAY_DOWNSAMPLE_FACTOR;
    int behavior_image_display_height =
        recorder_config.get_parameter<int>("behavior_camera", "roi_width") /
        DISPLAY_DOWNSAMPLE_FACTOR;
    int muscle_image_display_width =
        full_muscle_image_height / DISPLAY_DOWNSAMPLE_FACTOR;
    int muscle_image_display_height =
        full_muscle_image_width / DISPLAY_DOWNSAMPLE_FACTOR;

    // Create named windows
    cv::namedWindow("Behavior Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow(
        "Behavior Camera",
        behavior_image_display_width,
        behavior_image_display_height);

    cv::namedWindow("Muscle Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow(
        "Muscle Camera",
        muscle_image_display_width,
        muscle_image_display_height);

    // Add callback for muscle camera window
    cv::setMouseCallback("Muscle Camera", on_mouse, nullptr);
}
} // namespace

void align_camera(const std::filesystem::path &profile_dir) {
    std::filesystem::path config_path = profile_dir / "recorder_config.yaml";
    spdlog::info(
        "align-cameras loading recorder configuration from {}",
        config_path.string());
    RecorderConfig recorder_config(config_path);

    // Load muscle camera full frame size
    full_muscle_image_width =
        recorder_config.get_parameter<int>("muscle_camera", "full_frame_width");
    full_muscle_image_height = recorder_config.get_parameter<int>(
        "muscle_camera", "full_frame_height");
    muscle_image_roi_width =
        recorder_config.get_parameter<int>("muscle_camera", "roi_width");
    muscle_image_roi_height =
        recorder_config.get_parameter<int>("muscle_camera", "roi_height");

    // Load the normalization window for displaying the 16-bit muscle image
    // during camera alignment (separate from the main GUI's histogram
    // defaults).
    int muscle_display_vmin = recorder_config.get_parameter<int>(
        "muscle_camera", "align_cameras_display_vmin");
    int muscle_display_vmax = recorder_config.get_parameter<int>(
        "muscle_camera", "align_cameras_display_vmax");

    // Set up shared recording states
    std::shared_ptr<ProgramState> program_state =
        std::make_shared<ProgramState>();
    std::shared_ptr<ProgrammedStop> programmed_recording_stop =
        std::make_shared<ProgrammedStop>();
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state =
        std::make_shared<BehaviorRecordingState>();
    std::shared_ptr<MuscleRecordingState> muscle_recording_state =
        std::make_shared<MuscleRecordingState>();

    // Set up cameras acquisition threads
    spdlog::info("Starting behavior camera acquisition thread");
    behavior_recording_state->latest_frame_holder =
        std::make_shared<LatestFrame>();
    std::thread behavior_image_acquirer_thread(
        behavior_image_acquirer,
        recorder_config,
        behavior_recording_state,
        program_state,
        programmed_recording_stop);
    spdlog::info("Behavior camera acquisition thread started");

    spdlog::info("Setting up muscle camera acquisition thread");
    muscle_recording_state->latest_frame_holder =
        std::make_shared<LatestFrame>();
    std::thread muscle_image_acquirer_thread(
        muscle_image_acquirer,
        full_muscle_image_width,
        full_muscle_image_height,
        0, // x_offset
        0, // y_offset
        recorder_config,
        profile_dir,
        spdlog::get_level(),
        muscle_recording_state,
        program_state,
        programmed_recording_stop);
    spdlog::info("Muscle camera acquisition thread started");

    // Start Arduino triggering interface set default triggering parameters
    size_t retry_count = 0;
    while (!muscle_recording_state->muscle_camera.load()) {
        spdlog::debug("Waiting for muscle camera to be ready");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retry_count++;
        if (retry_count % 10 == 0) {
            spdlog::warn("Muscle camera is not initialized.");
        }
    }
    int muscle_num_lines_scanned =
        muscle_recording_state->muscle_camera.load()->get_num_lines_scanned();
    arduino_communication = initialize_triggering_with_default_params(
        recorder_config,
        muscle_num_lines_scanned,
        1,     // sync ratio
        true); // muscle_imaging_on

    // Set up display windows
    setup_display_windows(recorder_config);

    // Streaming loop
    spdlog::info("Starting streaming loop");
    cv::Mat behavior_image;
    cv::Mat muscle_image;
    cv::Mat behavior_image_display;
    cv::Mat muscle_image_display;
    while (true) {
        // Fetch latest images from both cameras
        behavior_image = behavior_recording_state->latest_frame_holder
                             ->get_latest_frame_data()
                             .image;
        muscle_image =
            muscle_recording_state->latest_frame_holder->get_latest_frame_data()
                .image;

        // Skip display if images are empty
        if (behavior_image.empty()) {
            spdlog::warn(
                "Behavior image is empty. Skipping display. This is normal "
                "if it only happens a few times at the beginning of the "
                "program while the camera initializes.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        if (muscle_image.empty()) {
            spdlog::warn(
                "Muscle image is empty. Skipping display. This is normal "
                "if it only happens a few times at the beginning of the "
                "program while the camera initializes.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        // Resize images for display
        cv::Size target_size;
        target_size = cv::Size(
            behavior_image.cols / DISPLAY_DOWNSAMPLE_FACTOR,
            behavior_image.rows / DISPLAY_DOWNSAMPLE_FACTOR);
        cv::resize(behavior_image, behavior_image_display, target_size);
        reorient_behavior_image(behavior_image_display, behavior_image_display);
        // Add a cross to the middle of the behavior image to help with
        // alignment
        add_cross_to_behavior_image(behavior_image_display);

        convert16_bit_to8_bit(
            muscle_image,
            muscle_image,
            muscle_display_vmin,
            muscle_display_vmax);
        target_size = cv::Size(
            muscle_image.cols / DISPLAY_DOWNSAMPLE_FACTOR,
            muscle_image.rows / DISPLAY_DOWNSAMPLE_FACTOR);
        cv::resize(muscle_image, muscle_image_display, target_size);
        reorient_muscle_image(muscle_image_display, muscle_image_display);
        cv::cvtColor(
            muscle_image_display, muscle_image_display, cv::COLOR_GRAY2BGR);

        // Draw markers on the displayed image to indicate ROI
        draw_muscle_image_roi(muscle_image_display);

        // Display images
        cv::imshow("Behavior Camera", behavior_image_display);
        cv::imshow("Muscle Camera", muscle_image_display);
        int pressed_key = cv::waitKey(1);
        if (pressed_key == 27) {
            // ESC key pressed
            break;
        }
        if (pressed_key == 13) {
            // Enter key pressed
            if (try_save_selected_roi(profile_dir)) {
                break;
            }
            continue;
        }
    }

    // Stop the cameras
    spdlog::info("Stopping behavior camera acquisition thread");
    program_state->to_quit.store(true);
    // Give some time for acquisition threads to break out of loop
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (std::shared_ptr<BehaviorCamera> behavior_camera =
            behavior_recording_state->behavior_camera.load()) {
        spdlog::info("Stopping acquisition on behavior camera");
        behavior_camera->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        behavior_recording_state->behavior_camera.store(nullptr);
    }
    muscle_recording_state->muscle_camera.store(nullptr);
    behavior_image_acquirer_thread.join();
    muscle_image_acquirer_thread.join();
    spdlog::info("Behavior camera acquisition thread stopped");

    // Stop triggering. The new protocol has no "stop triggering" command, so
    // switch the blue excitation light off, then close the link.
    spdlog::info("Switching off excitation and closing Arduino link");
    arduino_communication->stop_excitation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arduino_communication->stop_communication();
}

int main(int argc, char **argv) {
    CLIOptions options = parse_cli(argc, argv);

    spdlog::set_level(options.log_level);

    // Load recorder configuration
    profile_dir = std::filesystem::path(expand_path(options.profile_dir));

    align_camera(profile_dir);
    spdlog::info("Calibration procedure complete");

    return 0;
}