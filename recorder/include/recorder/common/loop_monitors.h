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
    // loopDescription: human-readable loop name used in the overrun warning,
    //   e.g. "Tracking controller thread". frequencyHz: target loop frequency;
    //   the minimum cycle period (the time budget for one iteration) is derived
    //   from it.
    LoopRateLimiter(std::string loopDescription, int frequencyHz);

    // Call once at the start of each loop iteration, before the work, to mark
    // the cycle's start time.
    void startCycle();

    // Call once at the end of each loop iteration, after the work. Sleeps for
    // whatever remains of the cycle; if the work already overran the cycle, logs
    // an overrun warning instead.
    void sleepUntilNextCycle();

  private:
    std::string loopDescription_;
    int frequencyHz_;
    uint64_t intervalUs_;

    uint64_t cycleStartTime_ = 0;
};
