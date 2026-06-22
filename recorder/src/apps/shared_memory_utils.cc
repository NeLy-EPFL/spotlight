#include "recorder/apps/shared_memory_utils.h"

namespace pco_shared_memory {

void setup_frame_data(
    const std::string &shm_frame_data_name,
    const size_t frame_buffer_size,
    uint8_t *&frame_data_ptr,
    bool create_new) {
    int shm_file_desc = -1;

    if (create_new) {

        shm_file_desc = shm_open(
            shm_frame_data_name.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shm_file_desc == -1) {
            std::string error_message =
                "Failed to open shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
        if (ftruncate(shm_file_desc, frame_buffer_size) == -1) {
            std::string error_message =
                "Failed to set size of shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
    }

    else {
        shm_file_desc = shm_open(shm_frame_data_name.c_str(), O_RDWR, 0666);
    }

    frame_data_ptr = (uint8_t *)mmap(
        nullptr,
        frame_buffer_size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shm_file_desc,
        0);
    if (frame_data_ptr == MAP_FAILED) {
        std::string error_message =
            "Failed to map shared memory for frame data: " +
            std::string(strerror(errno));
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    close(shm_file_desc);
}

void setup_shutter_open_time(
    const std::string &shm_shutter_open_time_name,
    unsigned int *&shutter_open_time_ptr,
    bool create_new) {
    int shm_file_desc = -1;
    size_t shutter_open_time_size = sizeof(unsigned int);

    if (create_new) {
        shm_file_desc = shm_open(
            shm_shutter_open_time_name.c_str(),
            O_CREAT | O_RDWR | O_TRUNC,
            0666);
        if (shm_file_desc == -1) {
            std::string error_message =
                "Failed to open shared memory for shutter-open time: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
        if (ftruncate(shm_file_desc, shutter_open_time_size) == -1) {
            std::string error_message =
                "Failed to set size of shared memory for shutter-open "
                "time: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
    } else {
        shm_file_desc =
            shm_open(shm_shutter_open_time_name.c_str(), O_RDWR, 0666);
    }

    shutter_open_time_ptr = (unsigned int *)mmap(
        nullptr,
        shutter_open_time_size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shm_file_desc,
        0);
    if (shutter_open_time_ptr == MAP_FAILED) {
        std::string error_message =
            "Failed to map shared memory for shutter-open time: " +
            std::string(strerror(errno));
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    close(shm_file_desc);
}

void setup_frame_metadata(
    const std::string &shm_frame_metadata_name,
    FrameMetadata *&frame_metadata_ptr,
    bool create_new) {
    int shm_file_desc = -1;

    if (create_new) {
        shm_file_desc = shm_open(
            shm_frame_metadata_name.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shm_file_desc == -1) {
            std::string error_message =
                "Failed to open shared memory for frame metadata: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
        if (ftruncate(shm_file_desc, sizeof(FrameMetadata)) == -1) {
            std::string error_message =
                "Failed to set size of shared memory for frame metadata: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
    } else {
        shm_file_desc = shm_open(shm_frame_metadata_name.c_str(), O_RDWR, 0666);
    }

    frame_metadata_ptr = (FrameMetadata *)mmap(
        nullptr,
        sizeof(FrameMetadata),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shm_file_desc,
        0);
    if (frame_metadata_ptr == MAP_FAILED) {
        std::string error_message =
            "Failed to map shared memory for frame metadata: " +
            std::string(strerror(errno));
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    close(shm_file_desc);
}

void setup_mutex(
    const std::string &shm_mutex_name,
    pthread_mutex_t *&mutex_ptr,
    bool create_new) {
    int shm_file_desc = -1;

    if (create_new) {
        shm_file_desc =
            shm_open(shm_mutex_name.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shm_file_desc == -1) {
            std::string error_message =
                "Failed to open shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
        if (ftruncate(shm_file_desc, sizeof(pthread_mutex_t)) == -1) {
            std::string error_message =
                "Failed to set size of shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
    } else {
        shm_file_desc = shm_open(shm_mutex_name.c_str(), O_RDWR, 0666);
    }

    mutex_ptr = (pthread_mutex_t *)mmap(
        nullptr,
        sizeof(pthread_mutex_t),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shm_file_desc,
        0);
    if (mutex_ptr == MAP_FAILED) {
        std::string error_message = "Failed to map shared memory for mutex: " +
                                    std::string(strerror(errno));
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    close(shm_file_desc);

    // First-time init for mutex
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(mutex_ptr, &attr);
}

void setup_condition_variable(
    const std::string &shm_cond_var_name,
    pthread_cond_t *&cond_var_ptr,
    bool create_new) {
    int shm_file_desc = -1;

    if (create_new) {
        shm_file_desc = shm_open(
            shm_cond_var_name.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shm_file_desc == -1) {
            std::string error_message =
                "Failed to open shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
        if (ftruncate(shm_file_desc, sizeof(pthread_cond_t)) == -1) {
            std::string error_message =
                "Failed to set size of shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(error_message);
            throw std::runtime_error(error_message);
        }
    } else {
        shm_file_desc = shm_open(shm_cond_var_name.c_str(), O_RDWR, 0666);
    }

    cond_var_ptr = (pthread_cond_t *)mmap(
        nullptr,
        sizeof(pthread_cond_t),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shm_file_desc,
        0);
    if (cond_var_ptr == MAP_FAILED) {
        std::string error_message =
            "Failed to map shared memory for frame count: " +
            std::string(strerror(errno));
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    close(shm_file_desc);

    // First-time init for condition variable
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    int err = pthread_cond_init(cond_var_ptr, &attr);
    if (err != 0) {
        std::string msg = "Failed to initialize condition variable: " +
                          std::string(strerror(err));
        spdlog::critical(msg);
        throw std::runtime_error(msg);
    }
}
} // namespace pco_shared_memory