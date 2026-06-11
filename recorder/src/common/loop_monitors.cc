#include "recorder/common/loop_monitors.h"

#include <chrono>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "recorder/common/utils.h"

SaverPerfTracker::SaverPerfTracker(
    std::string thread_description,
    std::string save_unit_description,
    std::string thread_id_string)
    : thread_description_(std::move(thread_description)),
      save_unit_description_(std::move(save_unit_description)),
      thread_id_string_(std::move(thread_id_string)) {
    start_window(get_current_time_microseconds(), first_window_us);
}

void SaverPerfTracker::update_recording_state(bool is_recording) {
    if (is_recording && !was_recording_) {
        start_window(get_current_time_microseconds(), first_window_us);
    }
    was_recording_ = is_recording;
}

void SaverPerfTracker::record_save(
    uint64_t save_duration_us, int queue_length) {
    save_time_sum_us_ += save_duration_us;
    queue_length_sum_ += queue_length;
    save_count_++;
    uint64_t now = get_current_time_microseconds();
    if (now >= window_end_time_) {
        spdlog::info(
            "{} (thread ID {}), mean over past {:.0f} s: {:.1f} frames in "
            "queue, {} us to save each {}",
            thread_description_,
            thread_id_string_,
            (now - window_start_time_) / 1e6,
            static_cast<double>(queue_length_sum_) / save_count_,
            save_time_sum_us_ / save_count_,
            save_unit_description_);
        start_window(now, window_us);
    }
}

void SaverPerfTracker::start_window(uint64_t start_time, uint64_t duration_us) {
    window_start_time_ = start_time;
    window_end_time_ = start_time + duration_us;
    save_time_sum_us_ = 0;
    queue_length_sum_ = 0;
    save_count_ = 0;
}

LoopRateLimiter::LoopRateLimiter(std::string loop_description, int frequency_hz)
    : loop_description_(std::move(loop_description)),
      frequency_hz_(frequency_hz), interval_us_(1'000'000 / frequency_hz) {}

void LoopRateLimiter::start_cycle() {
    cycle_start_time_ = get_current_time_microseconds();
}

void LoopRateLimiter::sleep_until_next_cycle() {
    uint64_t elapsed_us = get_current_time_microseconds() - cycle_start_time_;
    long int time_to_sleep_us = interval_us_ - elapsed_us;
    if (time_to_sleep_us > 0) {
        std::this_thread::sleep_for(
            std::chrono::microseconds(time_to_sleep_us));
    } else {
        spdlog::warn(
            "{} is running behind. I'm running this loop at {} Hz, so I have "
            "only {} us to complete each cycle. It took {} us this cycle. If "
            "this only happens sporadically, it's harmless.",
            loop_description_,
            frequency_hz_,
            interval_us_,
            elapsed_us);
    }
}
