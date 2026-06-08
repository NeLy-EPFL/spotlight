#include "recorder/common/loop_monitors.h"

#include <chrono>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "recorder/common/utils.h"

SaverPerfTracker::SaverPerfTracker(
    std::string threadDescription,
    std::string saveUnitDescription,
    std::string threadIdString)
    : threadDescription_(std::move(threadDescription)),
      saveUnitDescription_(std::move(saveUnitDescription)),
      threadIdString_(std::move(threadIdString)) {
    startWindow(getCurrentTimeMicroseconds(), firstWindowUs_);
}

void SaverPerfTracker::updateRecordingState(bool isRecording) {
    if (isRecording && !wasRecording_) {
        startWindow(getCurrentTimeMicroseconds(), firstWindowUs_);
    }
    wasRecording_ = isRecording;
}

void SaverPerfTracker::recordSave(uint64_t saveDurationUs, int queueLength) {
    saveTimeSumUs_ += saveDurationUs;
    queueLengthSum_ += queueLength;
    saveCount_++;
    uint64_t now = getCurrentTimeMicroseconds();
    if (now >= windowEndTime_) {
        spdlog::info(
            "{} (thread ID {}), mean over past {:.0f} s: {:.1f} frames in "
            "queue, {} us to save each {}",
            threadDescription_,
            threadIdString_,
            (now - windowStartTime_) / 1e6,
            static_cast<double>(queueLengthSum_) / saveCount_,
            saveTimeSumUs_ / saveCount_,
            saveUnitDescription_);
        startWindow(now, windowUs_);
    }
}

void SaverPerfTracker::startWindow(uint64_t startTime, uint64_t durationUs) {
    windowStartTime_ = startTime;
    windowEndTime_ = startTime + durationUs;
    saveTimeSumUs_ = 0;
    queueLengthSum_ = 0;
    saveCount_ = 0;
}

LoopRateLimiter::LoopRateLimiter(std::string loopDescription, int frequencyHz)
    : loopDescription_(std::move(loopDescription)),
      frequencyHz_(frequencyHz),
      intervalUs_(1'000'000 / frequencyHz) {}

void LoopRateLimiter::startCycle() {
    cycleStartTime_ = getCurrentTimeMicroseconds();
}

void LoopRateLimiter::sleepUntilNextCycle() {
    uint64_t elapsedUs = getCurrentTimeMicroseconds() - cycleStartTime_;
    long int timeToSleepUs = intervalUs_ - elapsedUs;
    if (timeToSleepUs > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(timeToSleepUs));
    } else {
        spdlog::warn(
            "{} is running behind. I'm running this loop at {} Hz, so I have "
            "only {} us to complete each cycle. It took {} us this cycle. If "
            "this only happens sporadically, it's harmless.",
            loopDescription_,
            frequencyHz_,
            intervalUs_,
            elapsedUs);
    }
}
