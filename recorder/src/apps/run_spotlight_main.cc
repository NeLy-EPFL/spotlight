/**
 * run-spotlight -- main recording application.
 *
 * Loads the recorder config from <profile_dir>/recorder_config.yaml, the
 * muscle-camera ROI from <profile_dir>/muscle_camera_roi.yaml, and the
 * arena registration model from <arena_dir>/model/calibration_result.yaml.
 * Reads arena dimensions from <arena_dir>/metadata.yaml to derive the stage
 * range used by the motion-stage preview widget.
 *
 * Launches all acquisition, saving, tracking, and Arduino threads and opens the
 * main GUI window. Whether muscle is imaged, and the muscle/behavior frame rates
 * and exposures, are controlled from the main GUI window at runtime.
 *
 * CLI:  run-spotlight -p PROFILE_DIR -a ARENA_DIR [OPTIONS]
 *       (see --help for details; -a/--arena is required)
 */
#include "recorder/apps/run_spotlight.h"

namespace {
QApplication *application = nullptr;
MainGUIWindow *mainGUIWindow = nullptr;
std::shared_ptr<ProgramState> programState;
std::shared_ptr<BehaviorRecordingState> behaviorRecordingState;
std::shared_ptr<MuscleRecordingState> muscleRecordingState;
std::shared_ptr<ArduinoCommunication> arduinoCommunication;

// Set by the SIGINT handler (async-signal-safe: it only stores to an atomic).
// A QTimer on the main thread polls it and closes the window, so Ctrl-C and the
// GUI close button take the exact same shutdown path (closeEvent -> quitProgram)
// rather than running anything Qt-unsafe from signal context.
std::atomic<bool> sigintReceived{false};

// Stage range (mm) covering the arena, for the motion-stage preview widget.
struct StageRange {
    double minXMm, maxXMm, minYMm, maxYMm;
};

StageRange computeStageRangeFromArena(
    const std::filesystem::path &arenaDir,
    const RecorderConfig &recorderConfig,
    const CalibrationParams &behaviorCamCalibrationParams)
/**
 * Compute the stage range covering the arena, for the motion-stage preview
 * widget. Read arena dimensions from <arenaDir>/metadata.yaml, then invert the
 * calibration model at the image center to find which stage position
 * corresponds to each of the four arena corners. The computed limits are
 * clipped to [0, physical_range_limit_mm].
 *
 * Side effect: registers the clipped limits as software motion stage limits via
 * setMotionStageLimits() so setTargetMotionStagePosition() clamps.
 */
{
    std::filesystem::path arenaMetadataPath = arenaDir / "metadata.yaml";
    if (!std::filesystem::exists(arenaMetadataPath)) {
        std::string errorMessage = fmt::format(
            "Arena metadata file not found: {}", arenaMetadataPath.string());
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }
    YAML::Node arenaMetadata = YAML::LoadFile(arenaMetadataPath.string());
    auto arenaDim = arenaMetadata["arena_dim"].as<std::vector<double>>();
    double arenaSizeXMm = arenaDim[0];
    double arenaSizeYMm = arenaDim[1];
    // Saved/registration images are rotated 90 deg CCW from the raw
    // sensor, so the image's column count equals the sensor ROI height
    // and its row count equals the ROI width.
    int roiWidth =
        recorderConfig.getParameter<int>("behavior_camera", "roi_width");
    int roiHeight =
        recorderConfig.getParameter<int>("behavior_camera", "roi_height");
    int imageCenterCol = roiHeight / 2;
    int imageCenterRow = roiWidth / 2;
    double stageMinXMm = std::numeric_limits<double>::infinity();
    double stageMaxXMm = -std::numeric_limits<double>::infinity();
    double stageMinYMm = std::numeric_limits<double>::infinity();
    double stageMaxYMm = -std::numeric_limits<double>::infinity();
    for (const auto &corner : std::vector<std::pair<double, double>>{
             {0.0, 0.0},
             {arenaSizeXMm, 0.0},
             {arenaSizeXMm, arenaSizeYMm},
             {0.0, arenaSizeYMm},
         }) {
        auto [sx, sy] =
            behaviorCamCalibrationParams.physicalPosAndPixelPosToStagePos(
                corner.first, corner.second, imageCenterRow, imageCenterCol);
        stageMinXMm = std::min(stageMinXMm, sx);
        stageMaxXMm = std::max(stageMaxXMm, sx);
        stageMinYMm = std::min(stageMinYMm, sy);
        stageMaxYMm = std::max(stageMaxYMm, sy);
    }
    spdlog::info(
        "Stage range covering arena {}x{} mm: X=[{:.3f}, {:.3f}], "
        "Y=[{:.3f}, {:.3f}] mm",
        arenaSizeXMm,
        arenaSizeYMm,
        stageMinXMm,
        stageMaxXMm,
        stageMinYMm,
        stageMaxYMm);

    // Clip computed limits to [0, physical_range_limit_mm] and register them
    // as software motion stage limits so setTargetMotionStagePosition() clamps.
    double physicalRangeLimitMm = recorderConfig.getParameter<double>(
        "motion_control", "physical_range_limit_mm");
    auto clipToPhysicalRange =
        [physicalRangeLimitMm](double val, const char *name) -> double {
        double clipped = std::clamp(val, 0.0, physicalRangeLimitMm);
        if (clipped != val) {
            spdlog::warn(
                "Software stage limit {} ({:.3f} mm) falls outside physical "
                "range [0, {:.3f}] mm; clamping to {:.3f} mm.",
                name,
                val,
                physicalRangeLimitMm,
                clipped);
        }
        return clipped;
    };
    stageMinXMm = clipToPhysicalRange(stageMinXMm, "stageMinX");
    stageMaxXMm = clipToPhysicalRange(stageMaxXMm, "stageMaxX");
    stageMinYMm = clipToPhysicalRange(stageMinYMm, "stageMinY");
    stageMaxYMm = clipToPhysicalRange(stageMaxYMm, "stageMaxY");
    setMotionStageLimits(stageMinXMm, stageMaxXMm, stageMinYMm, stageMaxYMm);

    return {stageMinXMm, stageMaxXMm, stageMinYMm, stageMaxYMm};
}

void joinIfJoinable(std::thread &thread, const char *name)
/**
 * Join `thread` if it is joinable, logging before and after.
 */
{
    spdlog::debug("Waiting for {} to finish", name);
    if (thread.joinable()) {
        thread.join();
    }
    spdlog::debug("{} finished", name);
}
} // namespace

bool quitProgram()
/**
 * Signal every worker thread to wind down, then return so the Qt event loop
 * exits and runSpotlightMain() can JOIN every thread before any hardware object
 * is destroyed.
 *
 * This only *signals* shutdown; it does not call std::exit() and does not join
 * here. Joining (and only then destroying the cameras) happens in
 * runSpotlightMain() after application->exec() returns. This ordering is what
 * makes the in-process muscle camera safe: the MuscleCamera object is touched
 * only by the muscle acquirer thread, so it must outlive that thread.
 *
 * Notes:
 *   - The muscle camera is stop()'d so a blocked waitForOneFrame() returns
 *     std::nullopt and the muscle acquirer breaks out of its loop even if no
 *     muscle frames are arriving.
 *   - The behavior camera is NOT stopped here: its acquirer self-terminates on
 *     toQuit (it stops its own grabber at loop exit). Its grab is bounded (a
 *     200 ms ScopedBuffer pop) and interruptible, so it observes toQuit and
 *     returns promptly even if no frames are arriving; the unified teardown in
 *     runSpotlightMain() additionally stop()s it before joining. (Only the blue
 *     excitation light is switched off below.)
 */
{
    spdlog::info("Shutdown requested. Initiating graceful shutdown");

    programState->toQuit.store(true);

    // Unblock the muscle acquirer's grab so it can observe toQuit and exit.
    if (muscleRecordingState->muscleCamera) {
        spdlog::info("Stopping acquisition on muscle camera");
        muscleRecordingState->muscleCamera->stop();
    }

    // Tell motion control request handler thread to stop
    spdlog::info("Telling motion control request handler thread to stop.");
    stopMotionControlRequestHandler(programState);

    // Tell behavior camera saver threads to stop
    spdlog::info("Telling behavior image saver threads to stop.");
    stopBehaviorImageSaver(behaviorRecordingState, programState);

    spdlog::info("Telling muscle image saver threads to stop.");
    stopMuscleImageSaver(muscleRecordingState, programState);

    // Stop muscle excitation
    spdlog::info("Switching off muscle excitation light.");
    arduinoCommunication->stopExcitation();

    // Let the Qt event loop terminate so exec() returns and the join/teardown in
    // runSpotlightMain() runs. (Closing the last window would also do this, but
    // calling quit() explicitly covers the SIGINT path too.)
    if (application) {
        application->quit();
    }
    return true;
}

int runSpotlightMain(int argc, char **argv) {
    // Async-signal-safe: only store to an atomic. A QTimer below polls it on the
    // main thread and routes Ctrl-C through the same close path as the GUI.
    std::signal(SIGINT, [](int) { sigintReceived.store(true); });

    // Parse command line arguments
    CLIOptions options = parseCLI(argc, argv);
    if (options.arenaDir.empty()) {
        std::cerr << "Error: -a/--arena is required (path to the arena "
                     "directory containing metadata.yaml and model/).\n";
        return 1;
    }

    // Set log level based on CLI options
    spdlog::set_level(options.logLevel);

    QApplication localApplication(argc, argv);
    application = &localApplication;

    // Make program state holder
    programState = std::make_shared<ProgramState>();

    // Load recorder configuration
    std::filesystem::path profileDir =
        std::filesystem::path(expandPath(options.profileDir));
    std::filesystem::path arenaDir =
        std::filesystem::path(expandPath(options.arenaDir));
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info(
        "runSpotlight main loading recorder configuration from {}",
        configPath.string());
    RecorderConfig recorderConfig(configPath);
    if (!recorderConfig.isDefined) {
        std::string errorMessage = fmt::format(
            "Failed to load recorder configuration. Cannot start recording. "
            "Expected valid recorder configuration file at {}",
            configPath.c_str());
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }

    // Load muscle ROI
    std::filesystem::path roiFilePath = profileDir / "muscle_camera_roi.yaml";
    MuscleCameraROI muscleROI = getMuscleCameraROI(roiFilePath);

    // Make atomic variable that holds the save directory
    std::string defaultSaveDirectory =
        recorderConfig.getParameter<std::string>("gui", "default_save_dir");
    std::shared_ptr<SaveDirectory> saveDirectory =
        std::make_shared<SaveDirectory>(defaultSaveDirectory);

    // Load position mapping / calibration parameters from the arena
    // registration scan (`fit-arena-registration` output).
    std::filesystem::path calibrationParamsFilePath =
        arenaDir / "model" / "calibration_result.yaml";
    spdlog::info(
        "Loading spatial calibration parameters from {}",
        calibrationParamsFilePath.string());
    CalibrationParams behaviorCamCalibrationParams(
        calibrationParamsFilePath.string());
    if (!behaviorCamCalibrationParams.isDefined) {
        std::string errorMessage = fmt::format(
            "Spatial calibration data not found or malformed. This is required "
            "for tracking and recording. Expected valid calibration file at {} "
            "(produced by `fit-arena-registration -a <arena_dir>`).",
            calibrationParamsFilePath.string());
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }

    // Load the active-area mask for closed-loop tracking.
    double boundaryMarginMm =
        recorderConfig.getParameter<double>("tracking", "boundary_margin_mm");
    ActiveAreaMask activeAreaMask(
        arenaDir.string(),
        boundaryMarginMm,
        behaviorCamCalibrationParams.stageAndPixelToPhysical);
    spdlog::info("Loaded active area mask from {}", arenaDir.string());

    // Compute the stage range covering the arena, for the motion-stage preview
    // widget. Also registers software motion stage limits as a side effect.
    StageRange stageRange = computeStageRangeFromArena(
        arenaDir, recorderConfig, behaviorCamCalibrationParams);

    // Initialize behavior and muscle imaging states
    behaviorRecordingState = std::make_shared<BehaviorRecordingState>();
    behaviorRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();
    muscleRecordingState = std::make_shared<MuscleRecordingState>();
    muscleRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();

    // Start tracking & motion control threads
    std::shared_ptr<TrackingControlState> trackingControlState =
        std::make_shared<TrackingControlState>();
    std::thread motionControlIOThread(
        motionControlRequestHandler,
        recorderConfig,
        trackingControlState,
        programState);
    std::thread motionStagePositionLoggerThread(
        motionStagePositionLogger,
        recorderConfig,
        trackingControlState,
        saveDirectory,
        programState);
    std::thread trackingControllerThread(
        trackingController,
        recorderConfig,
        std::ref(activeAreaMask),
        behaviorRecordingState,
        trackingControlState,
        std::ref(behaviorCamCalibrationParams),
        programState);

    // Start behavior image acquirer
    std::shared_ptr<ProgrammedStop> programmedRecordingStop =
        std::make_shared<ProgrammedStop>();

    std::thread behaviorImageAcquirerThread(
        behaviorImageAcquirer,
        recorderConfig,
        behaviorRecordingState,
        programState,
        programmedRecordingStop);
    spdlog::info("Behavior camera acquisition thread started");

    // Start behavior image savers
    std::vector<std::thread> behaviorImageSaverThreads;
    int numBehaviorImageSaverThreads = recorderConfig.getParameter<int>(
        "behavior_camera", "num_image_saving_threads");
    for (int i = 0; i < numBehaviorImageSaverThreads; i++) {
        behaviorImageSaverThreads.push_back(std::thread(
            behaviorImageSaver,
            recorderConfig,
            behaviorRecordingState,
            saveDirectory,
            programState));
    }
    spdlog::info("Behavior camera saver threads started");

    // Initialize the behavior (Euresys) camera FULLY before starting the muscle
    // (PCO) camera thread. The two camera SDKs must not run their library init /
    // device discovery concurrently in this process: doing so was observed to
    // hang the Euresys GenTL discovery (it never completes), which then wedges
    // the behavior acquirer thread in its constructor -- so no behavior frames
    // ever arrive and quit hangs joining that thread. This could not happen when
    // the muscle camera ran as a separate process (PCO_InitializeLib ran in that
    // child, never overlapping Euresys discovery here). Serializing the two inits
    // restores that separation.
    spdlog::info(
        "Waiting for behavior camera to initialize before starting the muscle "
        "camera...");
    while ((!behaviorRecordingState->behaviorCamera ||
            !behaviorRecordingState->behaviorCamera->isReady()) &&
           !programState->toQuit.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // The behavior acquirer sets toQuit if the camera fails to open (so the wait
    // above does not spin forever). Track whether startup succeeded; on failure
    // we skip the rest of startup but still fall through to the unified
    // join/teardown below -- an early `return` here would destroy still-joinable
    // std::threads and std::terminate the process (which would also strand the
    // grabber). muscleImageAcquirerThread / muscleImageSaverThreads are declared
    // here so they are in scope at the joins even when startup is aborted before
    // they are created (a default-constructed thread is simply not joinable).
    bool startupOk = !programState->toQuit.load();
    std::thread muscleImageAcquirerThread;
    std::vector<std::thread> muscleImageSaverThreads;
    int result = 0;
    if (!startupOk) {
        spdlog::critical(
            "Behavior camera failed to initialize. Aborting startup.");
    }

    if (startupOk) {
        spdlog::info("Behavior camera initialized; starting muscle camera.");

        // Start muscle image acquirer
        spdlog::info(
            "Loaded muscle camera ROI from {}: x0={}, x1={}, y0={}, y1={} "
            "(xOffset={}, yOffset={}, imageWidth={}, imageHeight={})",
            roiFilePath.string(),
            muscleROI.x0,
            muscleROI.x1,
            muscleROI.y0,
            muscleROI.y1,
            muscleROI.xOffset,
            muscleROI.yOffset,
            muscleROI.imageWidth,
            muscleROI.imageHeight);
        muscleImageAcquirerThread = std::thread(
            muscleImageAcquirer,
            muscleROI.imageWidth,
            muscleROI.imageHeight,
            muscleROI.xOffset,
            muscleROI.yOffset,
            recorderConfig,
            muscleRecordingState,
            programState,
            programmedRecordingStop);
        spdlog::info("Muscle camera acquisition thread started");
        size_t retryCount = 0;
        while (!muscleRecordingState->muscleCamera &&
               !programState->toQuit.load()) {
            // Wait for the muscle camera to be initialized. Also bail out if the
            // acquirer failed to open the camera and requested a shutdown, so we
            // do not spin here forever.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            spdlog::warn("Waiting for muscle camera to be initialized...");
            retryCount++;
            if (retryCount % 10 == 0) {
                spdlog::warn("Muscle camera is not initialized.");
            }
        }
        if (programState->toQuit.load()) {
            spdlog::critical(
                "Muscle camera failed to initialize. Aborting startup.");
            startupOk = false;
        }
    }

    if (startupOk) {
        // The muscle camera's shutter-open window is configured from the main GUI
        // window (initialized to, and tracking, the muscle light-on time spin
        // box).

        // Start muscle image savers
        int numMuscleImageSaverThreads = recorderConfig.getParameter<int>(
            "muscle_camera", "num_image_saving_threads");
        for (int i = 0; i < numMuscleImageSaverThreads; i++) {
            muscleImageSaverThreads.push_back(std::thread(
                muscleImageSaver,
                recorderConfig,
                muscleRecordingState,
                saveDirectory,
                programState,
                5)); // cv::IMWRITE_TIFF_COMPRESSION_LZW
        }
        spdlog::info("Muscle camera saver threads started");

        // Start Arduino triggering interface
        std::string arduinoPortName = findArduinoPortName(recorderConfig);
        arduinoCommunication =
            std::make_shared<ArduinoCommunication>(arduinoPortName);

        // Reboot the trigger controller into a clean, known state at startup. The
        // comm thread waits for the reboot and reopens the port, so the GUI's
        // first STREAM (sent when the main window is constructed below) reaches
        // the freshly reset controller.
        arduinoCommunication->reset();

        // Create and show GUI
        MainGUIWindow localMainGUIWindow(
            recorderConfig,
            behaviorRecordingState,
            muscleRecordingState,
            trackingControlState,
            std::ref(behaviorCamCalibrationParams),
            saveDirectory,
            arduinoCommunication,
            programState,
            programmedRecordingStop,
            activeAreaMask,
            stageRange.minXMm,
            stageRange.maxXMm,
            stageRange.minYMm,
            stageRange.maxYMm,
            nullptr);
        mainGUIWindow = &localMainGUIWindow;
        mainGUIWindow->show();

        // Poll the SIGINT flag on the main thread and route Ctrl-C through the
        // same graceful close as the GUI's close button (closeEvent ->
        // quitProgram).
        QTimer sigintPollTimer;
        QObject::connect(&sigintPollTimer, &QTimer::timeout, [&]() {
            if (sigintReceived.load() && mainGUIWindow) {
                spdlog::info("SIGINT received; closing the main window.");
                mainGUIWindow->close();
            }
        });
        sigintPollTimer.start(100); // ms

        result = application->exec();
        mainGUIWindow = nullptr;
    }

    // Unified teardown -- reached on both a normal GUI close and a startup abort.
    // Signal shutdown and tell every worker thread to stop (interrupting the
    // camera grabs so the acquirers exit even if no frames are arriving), then
    // JOIN them all before destroying the cameras: the in-process cameras are
    // touched only by their acquirer threads and must outlive them. On the normal
    // path quitProgram() already signalled most of this; the calls here are
    // idempotent and also cover the startup-abort path (where quitProgram() never
    // ran).
    programState->toQuit.store(true);
    if (behaviorRecordingState->behaviorCamera) {
        behaviorRecordingState->behaviorCamera->stop();
    }
    if (muscleRecordingState->muscleCamera) {
        muscleRecordingState->muscleCamera->stop();
    }
    stopMotionControlRequestHandler(programState);
    stopBehaviorImageSaver(behaviorRecordingState, programState);
    stopMuscleImageSaver(muscleRecordingState, programState);

    // Wait for threads to finish
    joinIfJoinable(
        behaviorImageAcquirerThread, "behavior image acquirer thread");

    for (auto &thread : behaviorImageSaverThreads) {
        joinIfJoinable(thread, "one of the behavior image saver threads");
    }

    joinIfJoinable(muscleImageAcquirerThread, "muscle image acquirer thread");

    for (auto &thread : muscleImageSaverThreads) {
        joinIfJoinable(thread, "one of the muscle image saver threads");
    }

    joinIfJoinable(motionControlIOThread, "motion control IO thread");
    joinIfJoinable(
        motionStagePositionLoggerThread,
        "motion stage position logger thread");
    joinIfJoinable(trackingControllerThread, "tracking controller thread");

    // Every acquirer thread is now joined, so it is safe to destroy the cameras.
    // ~MuscleCamera stops the camera and tears down the PCO SDK; ~BehaviorCamera
    // releases the grabber.
    muscleRecordingState->muscleCamera = nullptr;
    behaviorRecordingState->behaviorCamera = nullptr;

    return startupOk ? result : 1;
}

int main(int argc, char **argv) {
    try {
        return runSpotlightMain(argc, argv);
    } catch (const std::exception &e) {
        spdlog::critical("Fatal startup error: {}", e.what());
        return 1;
    }
}