#pragma once

#include <fcntl.h>
#include <pthread.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include <spdlog/spdlog.h>

namespace pco_shared_memory {
struct FrameMetadata {
    // Index of the published frame, counting real frames from 0. The server
    // sets this to -1 before the first frame so the consumer can tell "no frame
    // published yet" apart from "frame 0" (signed, matching
    // FrameData::frame_id).
    long frame_count = -1;
    uint64_t acquisition_time = 0;
};

void setup_frame_data(
    const std::string &shm_frame_data_name,
    const size_t frame_buffer_size,
    uint8_t *&frame_data_ptr,
    bool create_new);
void setup_shutter_open_time(
    const std::string &shm_shutter_open_time_name,
    unsigned int *&shutter_open_time_ptr,
    bool create_new);
void setup_frame_metadata(
    const std::string &shm_frame_metadata_name,
    FrameMetadata *&frame_metadata_ptr,
    bool create_new);
void setup_mutex(
    const std::string &shm_mutex_name,
    pthread_mutex_t *&mutex_ptr,
    bool create_new);
void setup_condition_variable(
    const std::string &shm_cond_var_name,
    pthread_cond_t *&cond_var_ptr,
    bool create_new);
} // namespace pco_shared_memory
