// Small shared helpers for the recorder's GoogleTest suite: an RAII temporary
// directory and an RAII environment-variable override. Both are used by tests
// that touch the filesystem or path-expansion code (which reads $HOME).

#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>

namespace fs = std::filesystem;

// A unique temporary directory that is removed (recursively) on destruction.
class TempDir {
  public:
    TempDir() {
        std::random_device rd;
        path_ = fs::temp_directory_path() /
                ("recorder_tests_" + std::to_string(rd()) + "_" +
                 std::to_string(rd()));
        fs::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path_, ec); // best effort; never throw from a destructor
    }
    TempDir(const TempDir &) = delete;
    TempDir &operator=(const TempDir &) = delete;

    const fs::path &path() const { return path_; }
    fs::path file(const std::string &name) const { return path_ / name; }

  private:
    fs::path path_;
};

// Sets an environment variable for the lifetime of the guard, restoring the
// previous value (or unsetting it) on destruction. Keeps $HOME-dependent tests
// from leaking state into one another.
class EnvGuard {
  public:
    EnvGuard(const char *name, const std::string &value) : name_(name) {
        const char *previous = std::getenv(name);
        had_ = previous != nullptr;
        if (had_) {
            saved_ = previous;
        }
        ::setenv(name, value.c_str(), /*overwrite=*/1);
    }
    ~EnvGuard() {
        if (had_) {
            ::setenv(name_.c_str(), saved_.c_str(), 1);
        } else {
            ::unsetenv(name_.c_str());
        }
    }
    EnvGuard(const EnvGuard &) = delete;
    EnvGuard &operator=(const EnvGuard &) = delete;

  private:
    std::string name_;
    std::string saved_;
    bool had_;
};
