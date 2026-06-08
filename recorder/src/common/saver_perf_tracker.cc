#include "recorder/common/saver_perf_tracker.h"

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
