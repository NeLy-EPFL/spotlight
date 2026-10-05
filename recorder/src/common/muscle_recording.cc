#include "recorder/common/muscle_recording.h"

#include "recorder/common/loop_monitors.h"

void muscle_image_acquirer(
    const MuscleCameraROIs &rois,
    const RecorderConfig &recorder_config,
    const std::string &profile_dir,
    spdlog::level::level_enum log_level,
    const std::shared_ptr<MuscleRecordingState> &muscle_recording_state,
    const std::shared_ptr<ProgramState> &program_state,
    const std::shared_ptr<ProgrammedStop> &programmed_recording_stop) {
    spdlog::info("Muscle image acquirer thread started");

    // Create muscle camera
    double rolling_shutter_line_time_us = recorder_config.get_parameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    double sensor_readout_time_us = recorder_config.get_parameter<double>(
        "muscle_camera", "sensor_readout_time_us");
    auto muscle_camera = std::make_shared<MuscleCamera>(
        rois,
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
        FramePair frame_pair = muscle_camera->wait_for_next_frame_pair();
        if (frame_pair.calcium.image.empty() ||
            frame_pair.fiducial.image.empty()) {
            spdlog::error("muscle_image_acquirer thread got an empty image");
        }
        muscle_recording_state->latest_calcium_frame_holder
            ->set_latest_frame_data(frame_pair.calcium);
        muscle_recording_state->latest_fiducial_frame_holder
            ->set_latest_frame_data(frame_pair.fiducial);

        // Only save muscle frames when the current recording images the
        // muscle cameras. The cameras always free-run (they are never
        // TTL-triggered), so without this gate a behavior-only recording would
        // save their un-excited, un-synced continuous-mode frames.
        bool is_recording = program_state->is_recording.load() &&
                            program_state->muscle_imaging_enabled.load();
        int num_frames_expected =
            programmed_recording_stop->num_muscle_frames_expected;

        if (is_recording && !reached_programmed_stop) {
            if (current_frame_id == 0) {
                spdlog::info("First muscle frame of the recording received");
            }
            frame_pair.frame_id = current_frame_id++;
            frame_pair.calcium.frame_id = frame_pair.frame_id;
            frame_pair.fiducial.frame_id = frame_pair.frame_id;
            {
                std::lock_guard<std::mutex> lock(
                    muscle_recording_state->muscle_image_queue_mutex);
                muscle_recording_state->muscle_image_queue.push(
                    std::move(frame_pair));
            }
            muscle_recording_state->muscle_image_queue_cond_var.notify_one();

            // Stop exactly on the programmed frame count: once the last
            // expected frame has been enqueued, stop recording on our own so no
            // extra frames are saved. Nothing else to do here -- the behavior
            // acquirer notifies the GUI to finalize. The muscle cameras keep
            // free-running (they are never TTL-triggered); we simply stop
            // enqueueing their frames.
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
    const std::shared_ptr<SaveDirectory> &save_directory,
    const std::shared_ptr<ProgramState> &program_state,
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
    FramePair frame_pair;

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
            frame_pair = std::move(
                muscle_recording_state->muscle_image_queue.front());
            muscle_recording_state->muscle_image_queue.pop();
        }

        perf_tracker.update_recording_state(program_state->is_recording.load());

        start_time = get_current_time_microseconds();
        std::string filename_stem =
            "muscle_frame_" + fmt::format("{:09}", frame_pair.frame_id);
        std::filesystem::path muscle_save_dir =
            std::filesystem::path(save_directory->get_directory()) /
            "muscle_images";

        const std::pair<const char *, const FrameData &> camera_frames[] = {
            {"calcium", frame_pair.calcium}, {"fiducial", frame_pair.fiducial}};

        // Save one TIFF per camera
        for (const auto &[camera_name, frame_data] : camera_frames) {
            // Reorient image (rotate it so it's consistent with behavior image)
            cv::Mat reoriented_image;
            reorient_muscle_image(frame_data.image, reoriented_image);

            std::string image_path =
                muscle_save_dir / (filename_stem + "_" + camera_name + ".tif");
            try {
                cv::imwrite(image_path, reoriented_image, compression_params);
            } catch (const cv::Exception &ex) {
                spdlog::error("Exception saving image: {}", ex.what());
            }
        }

        // Save metadata of both cameras (one row per camera)
        std::string metadata_path = muscle_save_dir / (filename_stem + ".csv");
        std::ofstream metadata_file(metadata_path);
        if (!metadata_file.is_open()) {
            spdlog::error(
                "Failed to open metadata file: {}", metadata_path.c_str());
        } else {
            metadata_file << "frame_id,camera,acquired_time_us,received_time_us,"
                             "server_frame_count,recorder_image_number\n";
            for (const auto &[camera_name, frame_data] : camera_frames) {
                metadata_file << frame_pair.frame_id << "," << camera_name
                              << "," << frame_data.acquisition_time << ","
                              << frame_data.received_time << ","
                              << frame_data.server_frame_count << ","
                              << frame_data.recorder_image_number << "\n";
            }
            metadata_file.close();
        }

        perf_tracker.record_save(
            get_current_time_microseconds() - start_time, queue_length);
    }
}

void stop_muscle_image_saver(
    const std::shared_ptr<MuscleRecordingState> &muscle_recording_state,
    const std::shared_ptr<ProgramState> &program_state) {
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
