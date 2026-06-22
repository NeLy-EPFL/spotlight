#pragma once

#include <cstdint>
#include <string>

// Rolling-window performance tracker for the image saver threads.
//
// Logging every save would be too noisy, so each saver thread instead reports
// its mean save time and mean queue length averaged over a rolling window. The
// first window of a recording session is short (~2 s) so early problems (e.g.
// the disk falling behind) surface quickly; every window after that is ~1
// minute.
//
// The window is scoped to recording. update_recording_state() (re)starts it at
// the beginning of each recording session, so the idle streaming time before
// recording started is never folded into a report.
class SaverPerfTracker {
  public:
    // thread_description: human-readable saver name, e.g. "Behavior image
    //   saver thread". save_unit_description: what one save covers, used in
    //   the report, e.g. "frame" or "group of three frames".
    //   thread_id_string: this thread's id, for cross-referencing with other
    //   logs.
    SaverPerfTracker(
        std::string thread_description,
        std::string save_unit_description,
        std::string thread_id_string);

    // Call once per loop iteration after dequeuing a frame, before saving,
    // passing the current recording flag. Starts a fresh window the first time
    // recording becomes active so reports reflect recording only, not the idle
    // streaming time beforehand.
    void update_recording_state(bool is_recording);

    // Call once per loop iteration after a save completes, passing how long the
    // save took and the queue length observed when the frame was dequeued. The
    // save is accumulated into the current window; once the window elapses, its
    // mean performance is reported and a new window begins.
    void record_save(uint64_t save_duration_us, int queue_length);

  private:
    void start_window(uint64_t start_time, uint64_t duration_us);

    static constexpr uint64_t first_window_us = 2'000'000; // 2 seconds
    static constexpr uint64_t window_us = 60'000'000;      // 1 minute

    std::string thread_description_;
    std::string save_unit_description_;
    std::string thread_id_string_;

    uint64_t window_start_time_ = 0;
    uint64_t window_end_time_ = 0;
    uint64_t save_time_sum_us_ = 0;
    uint64_t queue_length_sum_ = 0;
    int save_count_ = 0;
    bool was_recording_ = false;
};

// Paces a periodic loop to a target frequency and warns when the loop body
// overruns its time budget.
//
// A loop that must run at a fixed rate (e.g. the tracking controller or the
// stage position logger) measures how long its work took this cycle, then
// sleeps for whatever remains of the cycle. If the work took longer than the
// whole cycle there is nothing left to sleep and the loop has fallen behind;
// that is reported as a warning. Sporadic overruns are harmless, so the message
// says so.
class LoopRateLimiter {
  public:
    // loop_description: human-readable loop name used in the overrun warning,
    //   e.g. "Tracking controller thread". frequency_hz: target loop
    //   frequency; the minimum cycle period (the time budget for one
    //   iteration) is derived from it.
    LoopRateLimiter(std::string loop_description, int frequency_hz);

    // Call once at the start of each loop iteration, before the work, to mark
    // the cycle's start time.
    void start_cycle();

    // Call once at the end of each loop iteration, after the work. Sleeps for
    // whatever remains of the cycle; if the work already overran the cycle,
    // logs an overrun warning instead.
    void sleep_until_next_cycle();

  private:
    std::string loop_description_;
    int frequency_hz_;
    uint64_t interval_us_;

    uint64_t cycle_start_time_ = 0;
};
