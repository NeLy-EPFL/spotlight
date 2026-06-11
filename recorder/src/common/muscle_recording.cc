#include "recorder/common/muscle_recording.h"

#include "recorder/common/loop_monitors.h"

MuscleCameraROI::MuscleCameraROI(int x0, int x1, int y0, int y1)
    : x0(x0), x1(x1), y0(y0), y1(y1), x_offset(x0 - 1), y_offset(y0 - 1),
      image_width(x1 - x0 + 1), image_height(y1 - y0 + 1) {}

bool MuscleCameraROI::is_within_bound(int full_width, int full_height) const {
    return (
        x0 > 0 && x1 <= full_width && y0 > 0 && y1 <= full_height && x0 < x1 &&
        y0 < y1);
}

int MuscleCameraROI::to_file(const std::filesystem::path &path) const {
    YAML::Node node;
    node["x0"] = x0;
    node["x1"] = x1;
    node["y0"] = y0;
    node["y1"] = y1;
    node["x_offset"] = x_offset;
    node["y_offset"] = y_offset;
    node["image_width"] = image_width;
    node["image_height"] = image_height;

    std::ofstream fout(path);
    if (!fout) {
        spdlog::error("Failed to open file: {}", path.string());
        return 1;
    }

    fout << node;
    fout.close();
    return 0;
}

std::tuple<int, int> MuscleCameraROI::get_center_xy() const {
    return std::make_tuple((x0 + x1) / 2, (y0 + y1) / 2);
}

MuscleCameraROI
get_muscle_camera_roi(const std::filesystem::path &roi_file_path) {
    YAML::Node node;
    try {
        node = YAML::LoadFile(roi_file_path.string());
    } catch (const YAML::Exception &e) {
        throw std::runtime_error(fmt::format(
            "Failed to load muscle camera ROI from {}: {}",
            roi_file_path.string(),
            e.what()));
    }

    auto read_int = [&](const char *key) {
        if (!node[key]) {
            throw std::runtime_error(fmt::format(
                "Muscle camera ROI file {} is missing key '{}'",
                roi_file_path.string(),
                key));
        }
        try {
            return node[key].as<int>();
        } catch (const YAML::Exception &e) {
            throw std::runtime_error(fmt::format(
                "Muscle camera ROI key '{}' in {} is not an int: {}",
                key,
                roi_file_path.string(),
                e.what()));
        }
    };

    int x0 = read_int("x0");
    int x1 = read_int("x1");
    int y0 = read_int("y0");
    int y1 = read_int("y1");
    int image_width = read_int("image_width");
    int image_height = read_int("image_height");
    MuscleCameraROI roi(x0, x1, y0, y1);

    spdlog::info(
        "Muscle camera ROI loaded from file: x0={}, x1={}, y0={}, y1={}; "
        "image_width={}, image_height={}",
        x0,
        x1,
        y0,
        y1,
        image_width,
        image_height);
    return roi;
}

void muscle_image_acquirer(
    unsigned int image_width,
    unsigned int image_height,
    unsigned int x_offset,
    unsigned int y_offset,
    const RecorderConfig &recorder_config,
    const std::string &profile_dir,
    spdlog::level::level_enum log_level,
    const std::shared_ptr<MuscleRecordingState>& muscle_recording_state,
    const std::shared_ptr<ProgramState>& program_state,
    const std::shared_ptr<ProgrammedStop>& programmed_recording_stop) {
    spdlog::info("Muscle image acquirer thread started");

    // Create muscle camera
    double rolling_shutter_line_time_us = recorder_config.get_parameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    double sensor_readout_time_us = recorder_config.get_parameter<double>(
        "muscle_camera", "sensor_readout_time_us");
    auto muscle_camera = std::make_shared<MuscleCamera>(
        image_width,
        image_height,
        x_offset,
        y_offset,
        rolling_shutter_line_time_us,
        sensor_readout_time_us,
        recorder_config,
        profile_dir,
        log_level);
    // Publish for the other threads; keep a local handle for this thread's hot
    // loop so it doesn't pay an atomic load per frame.
    muscle_recording_state->muscle_camera.store(muscle_camera);

    spdlog::info("Muscle camera configured. Entering frame grabbing loop...");
    long int current_frame_id = 0;
    // Set once this thread has enqueued exactly the programmed number of frames
    // and stopped recording on its own, so subsequent frames are discarded
    // until the GUI tears the recording down.
    bool reached_programmed_stop = false;

    while (!program_state->to_quit.load()) {
        FrameData frame_data = muscle_camera->wait_for_one_frame();
        if (frame_data.image.empty()) {
            spdlog::error("muscle_image_acquirer thread got an empty image");
        }
        muscle_recording_state->latest_frame_holder->set_latest_frame_data(
            frame_data);

        bool is_recording = program_state->is_recording.load();
        int num_frames_expected =
            programmed_recording_stop->num_muscle_frames_expected;

        if (is_recording && !reached_programmed_stop) {
            if (current_frame_id == 0) {
                spdlog::info("First muscle frame of the recording received");
            }
            frame_data.frame_id = current_frame_id++;
            {
                std::lock_guard<std::mutex> lock(
                    muscle_recording_state->muscle_image_queue_mutex);
                muscle_recording_state->muscle_image_queue.push(frame_data);
            }
            muscle_recording_state->muscle_image_queue_cond_var.notify_one();

            // Stop exactly on the programmed frame count: once the last
            // expected frame has been enqueued, stop recording on our own so no
            // extra frames are saved. Nothing else to do here -- the behavior
            // acquirer notifies the GUI to finalize, and the Arduino also stops
            // triggering the muscle camera by itself.
            if (num_frames_expected >= 0 &&
                current_frame_id == num_frames_expected) {
                reached_programmed_stop = true;
                spdlog::info(
                    "Muscle camera reached programmed stop after {} frames.",
                    num_frames_expected);
            }
        } else if (!is_recording) {
            // If we're not recording, we need to reset the frame ID
            // counter so that the next recording session starts at 0
            current_frame_id = 0;
            reached_programmed_stop = false;
        }
    }
}

void muscle_image_saver(
    const RecorderConfig &recorder_config,
    std::shared_ptr<MuscleRecordingState> muscle_recording_state,
    const std::shared_ptr<SaveDirectory>& save_directory,
    const std::shared_ptr<ProgramState>& program_state,
    int tiff_compression_method) {
    std::thread::id my_thread_id = std::this_thread::get_id();
    std::stringstream ss;
    ss << my_thread_id;
    std::string thread_id_string = ss.str();
    spdlog::info(
        "Muscle image saver thread started (thread ID {})", thread_id_string);

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_TIFF_COMPRESSION);
    compression_params.push_back(tiff_compression_method);

    int queue_length = -1;
    uint64_t start_time = 0;
    FrameData frame_data;

    SaverPerfTracker perf_tracker(
        "Muscle image saver thread", "frame", thread_id_string);

    while (!program_state->to_quit.load()) {
        {
            std::unique_lock<std::mutex> lock(
                muscle_recording_state->muscle_image_queue_mutex);
            muscle_recording_state->muscle_image_queue_cond_var.wait(
                lock, [&muscle_recording_state, program_state] {
                    return !muscle_recording_state->muscle_image_queue
                                .empty() ||
                           program_state->to_quit.load();
                });

            if (program_state->to_quit.load()) {
                spdlog::info(
                    "Muscle image saver thread is breaking out of loop.");
                break;
            }

            queue_length = muscle_recording_state->muscle_image_queue.size();
            frame_data = muscle_recording_state->muscle_image_queue.front();
            muscle_recording_state->muscle_image_queue.pop();
        }

        perf_tracker.update_recording_state(program_state->is_recording.load());

        start_time = get_current_time_microseconds();
        std::string filename_stem =
            "muscle_frame_" + fmt::format("{:09}", frame_data.frame_id);
        std::filesystem::path muscle_save_dir =
            std::filesystem::path(save_directory->get_directory()) /
            "muscle_images";

        // Reorient image (rotate it so it's consistent with behavior image)
        cv::Mat reoriented_image;
        reorient_muscle_image(frame_data.image, reoriented_image);

        // Save image
        std::string image_path = muscle_save_dir / (filename_stem + ".tif");
        try {
            cv::imwrite(image_path, reoriented_image, compression_params);
        } catch (const cv::Exception &ex) {
            spdlog::error("Exception saving image: {}", ex.what());
        }

        // Save metadata
        std::string metadata_path = muscle_save_dir / (filename_stem + ".csv");
        std::ofstream metadata_file(metadata_path);
        if (!metadata_file.is_open()) {
            spdlog::error(
                "Failed to open metadata file: {}", metadata_path.c_str());
        } else {
            metadata_file << "frame_id,acquired_time_us,received_time_us\n";
            metadata_file << frame_data.frame_id << ","
                          << frame_data.acquisition_time << ","
                          << frame_data.received_time << "\n";
            metadata_file.close();
        }

        perf_tracker.record_save(
            get_current_time_microseconds() - start_time, queue_length);
    }
}

void stop_muscle_image_saver(
    const std::shared_ptr<MuscleRecordingState>& muscle_recording_state,
    const std::shared_ptr<ProgramState>& program_state) {
    if (!program_state->to_quit.load()) {
        spdlog::critical("stop_muscle_image_saver() called but to_quit is "
                         "not set to true. This shouldn't happen.");
        throw std::runtime_error(
            "stop_muscle_image_saver() called but to_quit is "
            "not set to true. This shouldn't happen.");
    } else {
        muscle_recording_state->muscle_image_queue_cond_var.notify_all();
    }
}
