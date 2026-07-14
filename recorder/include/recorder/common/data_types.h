#pragma once

#include <atomic>
#include <opencv2/opencv.hpp>

struct FrameData {
    // Matches the acquirers' frame counters (signed long); -1 until the frame
    // is assigned an id during recording.
    long frame_id = -1;
    // Camera-side acquisition timestamp: the Euresys grabber timestamp for the
    // behavior camera, and the PCO camera's own metadata timestamp
    // (microseconds since midnight) for the muscle camera.
    uint64_t acquisition_time = 0;
    // Host time (get_current_time_microseconds) when the frame was read out.
    uint64_t received_time = 0;
    // Muscle camera only: the PCO recorder's running image number for this frame
    // (pco::Image::getRecorderImageNumber()); 0 for behavior frames, which do
    // not set it.
    uint32_t pco_record_id = 0;
    cv::Mat image;
};

struct BehaviorCameraROI {
    unsigned int image_width;
    unsigned int image_height;
    unsigned int x_offset;
    unsigned int y_offset;
};

struct GroupOfThreeFrames {
    FrameData frame0;
    FrameData frame1;
    FrameData frame2;
    // Number of frames actually acquired (1, 2, or 3). A recording whose total
    // behavior-frame count is not a multiple of three ends on a partial group:
    // the unused trailing channels are saved black and not logged in the CSV.
    int num_valid_frames = 3;
};

struct SerialPortInfo {
    std::string port_name;
    std::string description;
    std::string manufacturer;
};

// Request type enum
enum MotionStageRequestType {
    get_current_position,
    set_target_position,
    wait_until_idle,
    check_if_idle,
    start_homing
};

// Position structure
enum PositionType { absolute, relative };

struct MotionStagePosition {
    double x_pos_mm;
    double y_pos_mm;
    PositionType position_type;
};

// Single request and response structure for all request types
struct MotionStageRequest {
    size_t client_id_hash;
    MotionStageRequestType request_type;
    MotionStagePosition position;
    float velocity;
    // Per-move acceleration (mm/s^2). 0 means "use the device-configured
    // acceleration" (see MotionControl::move_absolute).
    float acceleration = 0;
};

struct MotionStageResponse {
    MotionStagePosition position = MotionStagePosition{
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN()};
    bool is_idle = false;
    bool set_success = false;
};

enum CalibrationScanDirection { row_by_row, column_by_column };

struct ProgramState {
    std::atomic<bool> to_quit = false;
    std::atomic<bool> is_recording = false;
    // Whether the current recording images the muscle camera. When false the
    // muscle camera still free-runs (it is never TTL-triggered) but its frames
    // are not saved -- a behavior-only recording. Set by the GUI before
    // is_recording is raised; read by the muscle image acquirer.
    std::atomic<bool> muscle_imaging_enabled = false;
};

struct ProgrammedStop {
    int num_behavior_frames_expected = -1;
    int num_muscle_frames_expected = -1;
    // Raised by the behavior acquirer once it has recorded exactly the
    // programmed number of frames and stopped on its own. It signals the GUI to
    // finalize the recording (revert the UI and cameras to streaming); by the
    // time the GUI acts, the saved frame count is already exact.
    std::atomic<bool> programmed_stop_reached = false;
};
