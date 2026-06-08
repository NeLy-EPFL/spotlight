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
// The window is scoped to recording. updateRecordingState() (re)starts it at
// the beginning of each recording session, so the idle streaming time before
// recording started is never folded into a report.
class SaverPerfTracker {
  public:
    // threadDescription: human-readable saver name, e.g. "Behavior image saver
    //   thread". saveUnitDescription: what one save covers, used in the report,
    //   e.g. "frame" or "group of three frames". threadIdString: this thread's
    //   id, included in the report for cross-referencing with other logs.
    SaverPerfTracker(
        std::string threadDescription,
        std::string saveUnitDescription,
        std::string threadIdString);

    // Call once per loop iteration after dequeuing a frame, before saving,
    // passing the current recording flag. Starts a fresh window the first time
    // recording becomes active so reports reflect recording only, not the idle
    // streaming time beforehand.
    void updateRecordingState(bool isRecording);

    // Call once per loop iteration after a save completes, passing how long the
    // save took and the queue length observed when the frame was dequeued. The
    // save is accumulated into the current window; once the window elapses, its
    // mean performance is reported and a new window begins.
    void recordSave(uint64_t saveDurationUs, int queueLength);

  private:
    void startWindow(uint64_t startTime, uint64_t durationUs);

    static constexpr uint64_t firstWindowUs_ = 2'000'000;  // 2 seconds
    static constexpr uint64_t windowUs_ = 60'000'000;      // 1 minute

    std::string threadDescription_;
    std::string saveUnitDescription_;
    std::string threadIdString_;

    uint64_t windowStartTime_ = 0;
    uint64_t windowEndTime_ = 0;
    uint64_t saveTimeSumUs_ = 0;
    uint64_t queueLengthSum_ = 0;
    int saveCount_ = 0;
    bool wasRecording_ = false;
};
