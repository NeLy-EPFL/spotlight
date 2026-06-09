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

// Stage range (mm) covering the arena, for the motion-stage preview widget.
struct StageRange {
    double minXMm, maxXMm, minYMm, maxYMm;
};

StageRange computeStageRangeFromArena(
    const std::filesystem::path &arenaDir,
    RecorderConfig &recorderConfig,
    CalibrationParams &behaviorCamCalibrationParams)
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
 * Quit gracefully by explicitly stopping acquisition on the behavior
 * camera* and telling saver threads that the work is done.
 *
 * * Without stopping acquisition explicitly, the frame grabber will
 * think the device is still busy the next time we run the program.
 */
{
    spdlog::info("SIGINT received. Initiating graceful shutdown");

    programState->toQuit.store(true);

    // Stop behavior camera acquisition
    if (behaviorRecordingState->behaviorCamera) {
        spdlog::info("Stopping acquisition on behavior camera");
        behaviorRecordingState->behaviorCamera->stop();
    }

    // Terminate PCO camera server. stop() is bounded (SIGTERM, then SIGKILL
    // after a grace period), so an unresponsive server can never block the
    // shutdown indefinitely.
    //
    // We deliberately do NOT reset muscleRecordingState->muscleCamera here: the
    // muscle acquirer thread dereferences that shared_ptr without taking its own
    // copy, so destroying the MuscleCamera now would be a use-after-free. Once
    // the server is gone the acquirer blocks forever in waitForOneFrame()'s
    // pthread_cond_wait, but it is abandoned at std::exit() below, with the
    // MuscleCamera object left alive and valid until the process exits.
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

    std::exit(0);
}

int runSpotlightMain(int argc, char **argv) {
    std::signal(SIGINT, [](int) { quitProgram(); });

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
    std::thread muscleImageAcquirerThread(
        muscleImageAcquirer,
        muscleROI.imageWidth,
        muscleROI.imageHeight,
        muscleROI.xOffset,
        muscleROI.yOffset,
        recorderConfig,
        profileDir,
        spdlog::get_level(),
        muscleRecordingState,
        programState,
        programmedRecordingStop);
    spdlog::info("Muscle camera acquisition thread started");
    size_t retryCount = 0;
    while (!muscleRecordingState->muscleCamera) {
        // Wait for the muscle camera to be initialized
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        spdlog::warn("Waiting for muscle camera to be initialized...");
        retryCount++;
        if (retryCount % 10 == 0) {
            spdlog::warn("Muscle camera is not initialized.");
        }
    }
    // The muscle camera's shutter-open window is configured from the main GUI
    // window (initialized to, and tracking, the muscle light-on time spin box).

    // Start muscle image savers
    std::vector<std::thread> muscleImageSaverThreads;
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
    // comm thread waits for the reboot and reopens the port, so the GUI's first
    // STREAM (sent when the main window is constructed below) reaches the freshly
    // reset controller.
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

    int result = application->exec();

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

    return result;
}

int main(int argc, char **argv) {
    try {
        return runSpotlightMain(argc, argv);
    } catch (const std::exception &e) {
        spdlog::critical("Fatal startup error: {}", e.what());
        return 1;
    }
}