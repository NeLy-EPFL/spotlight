#include "recorder/common/behavior_recording.h"

#include "recorder/common/loop_monitors.h"

void behavior_image_acquirer(
    const RecorderConfig &recorder_config,
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
    const std::shared_ptr<ProgramState>& program_state,
    const std::shared_ptr<ProgrammedStop>& programmed_recording_stop) {
    spdlog::info("Behavior image acquirer thread started");
    BehaviorCameraROI camera_roi =
        get_behavior_behavior_camera_roi(recorder_config);

    std::string frame_grabber_trigger_line =
        recorder_config.get_parameter<std::string>(
            "behavior_camera", "frame_grabber_trigger_line");
    auto behavior_camera = std::make_shared<BehaviorCamera>(
        camera_roi.image_width,
        camera_roi.image_height,
        camera_roi.x_offset,
        camera_roi.y_offset,
        frame_grabber_trigger_line);
    // Publish for the other threads; keep a local handle for this thread's hot
    // loop so it doesn't pay an atomic load per frame.
    behavior_recording_state->behavior_camera.store(behavior_camera);

    spdlog::info("Behavior camera configured");

    behavior_camera->start();
    spdlog::info("Behavior camera started");

    FrameData frame_data_buffer[3];
    size_t frame_data_buffer_index = 0;
    long int current_frame_id = 0;

    bool was_recording = false;
    // Set once this thread has acquired exactly the programmed number of frames
    // and stopped recording on its own, so subsequent frames are discarded
    // until the GUI tears the recording down.
    bool reached_programmed_stop = false;

    // Flush a partial group of one or two buffered frames as a single
    // pseudo-BGR image (the missing channel(s) are saved black and are not
    // logged in the CSV metadata; see make_pseudo_bgr_image_from_three_frames
    // and make_metadata_string_from_three_frames). Resets the buffer index.
    auto flush_partial_group = [&]() {
        GroupOfThreeFrames partial_group;
        partial_group.frame0 = frame_data_buffer[0];
        if (frame_data_buffer_index > 1) {
            partial_group.frame1 = frame_data_buffer[1];
        }
        partial_group.num_valid_frames =
            static_cast<int>(frame_data_buffer_index);
        {
            std::lock_guard<std::mutex> lock(
                behavior_recording_state->behavior_image_queue_mutex);
            behavior_recording_state->behavior_image_queue.push(partial_group);
        }
        behavior_recording_state->behavior_image_queue_cond_var.notify_one();
        frame_data_buffer_index = 0;
    };

    while (!program_state->to_quit.load()) {
        // Acquire image data
        // // Benchmark here shows that the wait_for_one_frame() function
        // // takes on average (triggering_cycle_period - 150) us to complete.
        // // So we have plenty of margin and can theoretically record at
        // // 1,000,000 / 200-ish = 5,000 fps.
        // // uint64_t start_time = get_current_time_microseconds();
        // spdlog::debug(
        //     "Behavior image acquirer thread waiting for one frame");
        FrameData frame_data = behavior_camera->wait_for_one_frame();
        // spdlog::debug(
        //     "Behavior image acquirer thread received one frame");
        // uint64_t wait_time = get_current_time_microseconds() - start_time;
        // spdlog::info("Behavior camera waited {} us", wait_time);

        // Update latest frame for live display
        behavior_recording_state->latest_frame_holder->set_latest_frame_data(
            frame_data);

        bool is_recording = program_state->is_recording.load();

        if (is_recording && !was_recording) {
            // Start of a new recording session: reset the counters.
            frame_data_buffer_index = 0;
            current_frame_id = 0;
            reached_programmed_stop = false;
            spdlog::info("First behavior frame of the recording received");
        }

        int num_frames_expected =
            programmed_recording_stop->num_behavior_frames_expected;

        if (is_recording && !reached_programmed_stop) {
            frame_data.frame_id = current_frame_id;

            frame_data_buffer[frame_data_buffer_index++] = frame_data;

            if (frame_data_buffer_index == 3) {
                // Add a full group of three frames to the queue.
                GroupOfThreeFrames group_of_three_frames = {
                    frame_data_buffer[0],
                    frame_data_buffer[1],
                    frame_data_buffer[2],
                    3};
                {
                    std::lock_guard<std::mutex> lock(
                        behavior_recording_state->behavior_image_queue_mutex);
                    behavior_recording_state->behavior_image_queue.push(
                        group_of_three_frames);
                }
                behavior_recording_state->behavior_image_queue_cond_var
                    .notify_one();

                frame_data_buffer_index = 0;
            }

            // Stop exactly on the programmed frame count. Once the last
            // expected frame has been acquired, flush any partial group and
            // stop recording right here, rather than waiting for the GUI to
            // tear the recording down (which would overrun by however many
            // frames arrive during the GUI's poll latency). The GUI is notified
            // via programmed_stop_reached so it can finalize the UI and revert
            // the cameras to streaming.
            if (num_frames_expected >= 0 &&
                current_frame_id == num_frames_expected - 1) {
                if (frame_data_buffer_index > 0) {
                    flush_partial_group();
                }
                reached_programmed_stop = true;
                programmed_recording_stop->programmed_stop_reached.store(true);
                spdlog::info(
                    "Programmed stop reached after {} behavior frames. "
                    "Behavior "
                    "acquisition thread stopped recording and is telling the "
                    "GUI to finalize.",
                    num_frames_expected);
            }

            current_frame_id++;
        } else {
            if (was_recording && !reached_programmed_stop &&
                frame_data_buffer_index > 0) {
                // A user-initiated stop landed on a partial group of one or two
                // frames; flush it. (A programmed stop has already flushed its
                // own partial group above.)
                flush_partial_group();
            }

            if (!is_recording) {
                // Not recording: any newly arrived frame is discarded (it is
                // only used for the live preview above). Reset the counters for
                // the next recording session.
                frame_data_buffer_index = 0;
                current_frame_id = 0;
                reached_programmed_stop = false;
            }
        }

        was_recording = is_recording;
    }

    // Stop behavior camera acquisition
    spdlog::info("Stopping acquisition on behavior camera");
    behavior_camera->stop();
    spdlog::info("Behavior camera acquisition stopped. "
                 "Behavior image acquirer thread reached its end");
}

void behavior_image_saver(
    const RecorderConfig &recorder_config,
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
    const std::shared_ptr<SaveDirectory>& save_directory,
    const std::shared_ptr<ProgramState>& program_state) {
    std::thread::id my_thread_id = std::this_thread::get_id();
    std::stringstream ss;
    ss << my_thread_id;
    std::string thread_id_string = ss.str();
    spdlog::info(
        "Behavior image saver thread started (thread ID {})", thread_id_string);

    // Define OpenCV JPEG saving parameters
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100); // Maximum quality, minimal compression
    compression_params.push_back(cv::IMWRITE_JPEG_CHROMA_QUALITY);
    compression_params.push_back(100); // Maximum quality, minimal compression
    // Unclear why OpenCV is built without this option. It should because I'm
    // on OpenCV 4.8 with libjpeg ver 80. This option directly specifies the
    // type of chroma subsampling used. This is about resolution of the chroma
    // channels rather than compression quality. Common values: 444 = No chroma
    // subsampling (full resolution for chroma channels)
    // compressionParams.push_back(cv::IMWRITE_JPEG_SAMPLING_FACTOR);
    // compressionParams.push_back(444); // Disable chroma subsampling (4:4:4)

    int queue_length = -1;

    SaverPerfTracker perf_tracker(
        "Behavior image saver thread",
        "group of three frames",
        thread_id_string);

    while (!program_state->to_quit.load()) {
        GroupOfThreeFrames frame_group;
        {
            std::unique_lock<std::mutex> lock(
                behavior_recording_state->behavior_image_queue_mutex);
            behavior_recording_state->behavior_image_queue_cond_var.wait(
                lock, [&behavior_recording_state, program_state] {
                    return !behavior_recording_state->behavior_image_queue
                                .empty() ||
                           program_state->to_quit.load();
                });

            if (program_state->to_quit.load()) {
                spdlog::info(
                    "Behavior image saver thread is breaking out of loop.");
                break;
            }

            queue_length =
                behavior_recording_state->behavior_image_queue.size();
            frame_group =
                behavior_recording_state->behavior_image_queue.front();
            behavior_recording_state->behavior_image_queue.pop();
        }

        perf_tracker.update_recording_state(program_state->is_recording.load());

        uint64_t start_time = get_current_time_microseconds();

        std::string filename_stem =
            "behavior_frame_" +
            fmt::format("{:09}", frame_group.frame0.frame_id);
        std::filesystem::path behavior_save_dir =
            std::filesystem::path(save_directory->get_directory()) /
            "behavior_images";

        // Save three frames as a single pseudo-BGR image
        std::string filename = behavior_save_dir / (filename_stem + ".jpg");
        cv::Mat image = make_pseudo_bgr_image_from_three_frames(frame_group);
        cv::imwrite(filename, image, compression_params);

        // Save metadata
        std::string metadata_filename =
            behavior_save_dir / (filename_stem + ".csv");
        std::ofstream metadata_file(metadata_filename);
        metadata_file << make_metadata_string_from_three_frames(frame_group);
        metadata_file.close();

        perf_tracker.record_save(
            get_current_time_microseconds() - start_time, queue_length);
    }
    spdlog::info("Behavior image saver thread stopped");
}

void stop_behavior_image_saver(
    const std::shared_ptr<BehaviorRecordingState>& behavior_recording_state,
    const std::shared_ptr<ProgramState>& program_state) {
    if (!program_state->to_quit.load()) {
        spdlog::critical("stop_behavior_image_saver() called but to_quit is "
                         "not set to true. This shouldn't happen.");
        throw std::runtime_error(
            "stop_behavior_image_saver() called but to_quit is "
            "not set to true. This shouldn't happen.");
    } else {
        behavior_recording_state->behavior_image_queue_cond_var.notify_all();
    }
}

BehaviorCameraROI
get_behavior_behavior_camera_roi(const RecorderConfig &recorder_config) {
    int image_width = round_to_nearest_valid_behavior_cam_dimension(
        recorder_config.get_parameter<int>("behavior_camera", "roi_width"));
    int image_height = round_to_nearest_valid_behavior_cam_dimension(
        recorder_config.get_parameter<int>("behavior_camera", "roi_height"));
    int full_frame_width = recorder_config.get_parameter<int>(
        "behavior_camera", "full_frame_width");
    int full_frame_height = recorder_config.get_parameter<int>(
        "behavior_camera", "full_frame_height");

    if (image_width < 0 || image_height < 0 || full_frame_width < 0 ||
        full_frame_height < 0 || image_width > full_frame_width ||
        image_height > full_frame_height) {
        std::string error_message = fmt::format(
            "Invalid camera ROI or full frame size: "
            "image_width = {}, image_height = {}, "
            "full_frame_width = {}, full_frame_height = {}",
            image_width,
            image_height,
            full_frame_width,
            full_frame_height);
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }

    auto [x_offset, y_offset] = get_centered_offsets(
        image_width, image_height, full_frame_width, full_frame_height);

    BehaviorCameraROI camera_roi = {
        (unsigned int)image_width,
        (unsigned int)image_height,
        (unsigned int)x_offset,
        (unsigned int)y_offset};
    return camera_roi;
}