#pragma once

#include <atomic>
#include <cstdint>
#include <fcntl.h>
#include <pthread.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include <spdlog/spdlog.h>

#include "recorder/common/recorder_config.h"

namespace pco_shared_memory {
// The two PCO muscle cameras, each served by its own pco-camera-server process.
enum class MuscleCameraRole { calcium, fiducial };

// "calcium" or "fiducial": the CLI value, and the suffix of the camera's keys in
// the muscle_camera section of the recorder config.
std::string role_to_string(MuscleCameraRole role);
// Inverse of role_to_string(). Throws std::invalid_argument for other strings.
MuscleCameraRole role_from_string(const std::string &name);

// Names of one camera's shared-memory regions, read from the recorder config
// (muscle_camera/shared_*_name_<role>).
struct SharedMemoryNames {
    std::string frame_data;
    std::string shutter_open_time;
    std::string server_state;
    std::string mutex;
    std::string condvar;
};
SharedMemoryNames get_shared_memory_names(
    const RecorderConfig &recorder_config, MuscleCameraRole role);

struct FrameMetadata {
    // Index of the published frame, counting real frames from 0. The server
    // sets this to -1 before the first frame so the consumer can tell "no frame
    // published yet" apart from "frame 0" (signed, matching
    // FrameData::frame_id). Counts frames the server fetched, not frames the
    // camera exposed: see recorder_image_number for the latter.
    long frame_count = -1;
    // Host time when the server fetched the frame (us since epoch).
    uint64_t acquisition_time = 0;
    // PCO recorder's count of images received since record() started. A jump
    // of more than one between published frames means the server missed some.
    uint32_t recorder_image_number = 0;
    // Image counter from the camera's binary timestamp.
    uint32_t camera_image_counter = 0;
    // Camera's binary timestamp (us since epoch, on the camera's own clock,
    // which is synchronized neither with the host nor with the other camera).
    uint64_t camera_timestamp_us = 0;
};

// State published by the camera server.
struct ServerState {
    // Set once the camera is configured and recording. Read by the client
    // without the mutex (it is only safe to use the mutex after this is set).
    std::atomic<bool> is_ready;
    // The most recently published frame. Protected by the mutex.
    FrameMetadata latest_frame;
};
static_assert(std::atomic<bool>::is_always_lock_free);

// Nominal exposure of the PCO camera (us). The client writes `requested_us`;
// the server applies it to the camera and then echoes it in `applied_us`, so
// the client can wait until the change has taken effect. Lock-free atomics, so
// no mutex is needed across processes.
struct ShutterOpenTime {
    std::atomic<unsigned int> requested_us;
    std::atomic<unsigned int> applied_us;
};
static_assert(std::atomic<unsigned int>::is_always_lock_free);

void setup_frame_data(
    const std::string &shm_frame_data_name,
    const size_t frame_buffer_size,
    uint8_t *&frame_data_ptr,
    bool create_new);
void setup_shutter_open_time(
    const std::string &shm_shutter_open_time_name,
    ShutterOpenTime *&shutter_open_time_ptr,
    bool create_new);
void setup_server_state(
    const std::string &shm_server_state_name,
    ServerState *&server_state_ptr,
    bool create_new);
void setup_mutex(
    const std::string &shm_mutex_name,
    pthread_mutex_t *&mutex_ptr,
    bool create_new);
void setup_condition_variable(
    const std::string &shm_condvar_name,
    pthread_cond_t *&condvar_ptr,
    bool create_new);
} // namespace pco_shared_memory
