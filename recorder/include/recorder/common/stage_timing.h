#pragma once

#include <cstdint>

#include "recorder/common/utils.h"

// PROFILING ONLY (headless-recorder-profiling branch): collects the duration of
// named pipeline stages and logs their distribution, to break down where time
// goes and the latency of each step.

// Clear all samples and start collecting (called when a recording starts).
void stage_timing_start();
// Record one sample (us) of `stage`, a string literal. No-op until started.
void stage_timing_record(const char *stage, int64_t duration_us);
// Log count / mean / p50 / p90 / p99 / max of each stage and stop collecting.
void stage_timing_log();

// Records the time from construction to destruction as one sample of `stage`.
struct ScopedStageTimer {
    const char *stage;
    uint64_t start_us = get_current_time_microseconds();
    ~ScopedStageTimer() {
        stage_timing_record(stage, get_current_time_microseconds() - start_us);
    }
};
