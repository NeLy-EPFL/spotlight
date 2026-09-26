#include "recorder/apps/shared_memory_utils.h"

#include <cerrno>
#include <cstring> // strerror
#include <stdexcept>
#include <sys/stat.h> // fstat

namespace pco_shared_memory {
namespace {
// Create (create_new = true) or attach to (create_new = false) the POSIX
// shared-memory region `name` of `size` bytes and map it read-write. When
// attaching, the region must already exist with at least `size` bytes: mapping
// a region that its creator has not sized yet would SIGBUS on first access.
// Throws std::runtime_error (mentioning `description`) on failure.
void *map_region(
    const std::string &name,
    size_t size,
    bool create_new,
    const std::string &description) {
    auto fail = [&](const std::string &what, int err) {
        throw std::runtime_error(
            "Failed to " + what + " shared memory for " + description + " (" +
            name + "): " + std::string(strerror(err)));
    };

    int flags = create_new ? (O_CREAT | O_RDWR | O_TRUNC) : O_RDWR;
    int fd = shm_open(name.c_str(), flags, 0666);
    if (fd == -1) {
        fail(create_new ? "create" : "open", errno);
    }

    if (create_new) {
        if (ftruncate(fd, size) == -1) {
            int err = errno;
            close(fd);
            fail("set size of", err);
        }
    } else {
        struct stat info;
        if (fstat(fd, &info) == -1) {
            int err = errno;
            close(fd);
            fail("stat", err);
        }
        if (static_cast<size_t>(info.st_size) < size) {
            close(fd);
            throw std::runtime_error(
                "Shared memory for " + description + " (" + name + ") has " +
                std::to_string(info.st_size) + " bytes, expected " +
                std::to_string(size));
        }
    }

    void *ptr =
        mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    int err = errno;
    close(fd);
    if (ptr == MAP_FAILED) {
        fail("map", err);
    }
    return ptr;
}
} // namespace

void setup_frame_data(
    const std::string &shm_frame_data_name,
    const size_t frame_buffer_size,
    uint8_t *&frame_data_ptr,
    bool create_new) {
    frame_data_ptr = static_cast<uint8_t *>(map_region(
        shm_frame_data_name, frame_buffer_size, create_new, "frame data"));
}

void setup_shutter_open_time(
    const std::string &shm_shutter_open_time_name,
    unsigned int *&shutter_open_time_ptr,
    bool create_new) {
    shutter_open_time_ptr = static_cast<unsigned int *>(map_region(
        shm_shutter_open_time_name,
        sizeof(unsigned int),
        create_new,
        "shutter-open time"));
}

void setup_frame_metadata(
    const std::string &shm_frame_metadata_name,
    FrameMetadata *&frame_metadata_ptr,
    bool create_new) {
    frame_metadata_ptr = static_cast<FrameMetadata *>(map_region(
        shm_frame_metadata_name,
        sizeof(FrameMetadata),
        create_new,
        "frame metadata"));
}

void setup_mutex(
    const std::string &shm_mutex_name,
    pthread_mutex_t *&mutex_ptr,
    bool create_new) {
    mutex_ptr = static_cast<pthread_mutex_t *>(map_region(
        shm_mutex_name, sizeof(pthread_mutex_t), create_new, "mutex"));

    // Only the creator initializes the mutex. Re-initializing a mutex that the
    // other process may already hold is undefined behavior.
    if (create_new) {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(mutex_ptr, &attr);
    }
}

void setup_condition_variable(
    const std::string &shm_cond_var_name,
    pthread_cond_t *&cond_var_ptr,
    bool create_new) {
    cond_var_ptr = static_cast<pthread_cond_t *>(map_region(
        shm_cond_var_name,
        sizeof(pthread_cond_t),
        create_new,
        "condition variable"));

    // Only the creator initializes the condition variable. Re-initializing one
    // that the other process may be waiting on is undefined behavior.
    if (create_new) {
        pthread_condattr_t attr;
        pthread_condattr_init(&attr);
        pthread_condattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        int err = pthread_cond_init(cond_var_ptr, &attr);
        if (err != 0) {
            throw std::runtime_error(
                "Failed to initialize condition variable: " +
                std::string(strerror(err)));
        }
    }
}
} // namespace pco_shared_memory
