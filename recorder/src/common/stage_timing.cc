#include "recorder/common/stage_timing.h"

#include <algorithm>
#include <atomic>
#include <map>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace {
std::atomic<bool> enabled = false;
std::mutex samples_mutex;
std::map<std::string, std::vector<int64_t>> samples;
} // namespace

void stage_timing_start() {
    std::lock_guard<std::mutex> lock(samples_mutex);
    samples.clear();
    enabled.store(true);
}

void stage_timing_record(const char *stage, int64_t duration_us) {
    if (!enabled.load()) {
        return;
    }
    std::lock_guard<std::mutex> lock(samples_mutex);
    samples[stage].push_back(duration_us);
}

void stage_timing_log() {
    enabled.store(false);
    std::lock_guard<std::mutex> lock(samples_mutex);
    for (auto &[stage, values] : samples) {
        std::sort(values.begin(), values.end());
        auto percentile = [&values](double p) {
            return values[static_cast<size_t>(p * (values.size() - 1))];
        };
        double mean =
            std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        spdlog::info(
            "STAGE_TIMING | {} | n={} mean={:.0f} p50={} p90={} p99={} max={}",
            stage,
            values.size(),
            mean,
            percentile(0.5),
            percentile(0.9),
            percentile(0.99),
            values.back());
    }
}
