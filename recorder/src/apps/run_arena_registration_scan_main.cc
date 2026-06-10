/**
 * run-arena-registration-scan
 *
 * Performs the arena registration scan needed to fit the
 * (stage, pixel) <-> physical mapping model.
 *
 * Workflow
 * --------
 * 1. Live preview with crosshairs -- the user centres the camera on the
 *    DataMatrix barcode printed on the mapping board, then presses ENTER.
 * 2. The DataMatrix is decoded and its checksum is verified against
 *    <arena_dir>/metadata.yaml to confirm the correct arena is mounted.
 * 3. The stage-to-arena offset is computed from the current stage position
 *    and the known DataMatrix position in arena coordinates.
 * 4. The stage visits each AprilTag position in sequence, acquires 10
 *    consecutive frames per tag (after dropping a configurable number of
 *    settling frames), and saves:
 *      <arena_dir>/mapping_scan/apriltag<id>_img<i>.jpg
 *      <arena_dir>/mapping_scan/apriltag_stage_positions.csv
 *
 * After the scan, run `fit-arena-registration -a <arena_dir>` to fit the
 * calibration model.
 *
 * CLI:  run-arena-registration-scan -p PROFILE_DIR -a ARENA_DIR [OPTIONS]
 *       (see --help for details)
 *
 * Requires libdmtx-dev: sudo apt install libdmtx-dev
 */
// Requires libdmtx-dev: sudo apt install libdmtx-dev
#include "recorder/apps/run_arena_registration_scan.h"

#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>

#include <dmtx.h>
#include <yaml-cpp/yaml.h>
#include <zaber/motion/exceptions/bad_data_exception.h>

namespace {
std::shared_ptr<ProgramState> programState = nullptr;
std::unique_ptr<ArduinoCommunication> arduinoCommunication = nullptr;
std::shared_ptr<BehaviorRecordingState> behaviorRecordingState = nullptr;

void quitProgram() {
    spdlog::info("SIGINT received. Initiating graceful shutdown");
    if (programState)
        programState->toQuit.store(true);
    if (arduinoCommunication) {
        // The new protocol has no "stop triggering" command; switch the blue
        // excitation light off, then close the link.
        arduinoCommunication->stopExcitation();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arduinoCommunication->stopCommunication();
    }
    std::exit(0);
}

// Single decode attempt at a given shrink factor.
std::string
tryDecodeDataMatrix(const cv::Mat &gray8u, int shrink, int timeoutMs) {
    DmtxImage *dmtxImg =
        dmtxImageCreate(gray8u.data, gray8u.cols, gray8u.rows, DmtxPack8bppK);
    if (!dmtxImg)
        return "";

    DmtxDecode *dec = dmtxDecodeCreate(dmtxImg, shrink);
    if (!dec) {
        dmtxImageDestroy(&dmtxImg);
        return "";
    }

    DmtxTime timeout = dmtxTimeAdd(dmtxTimeNow(), timeoutMs);
    DmtxRegion *reg = dmtxRegionFindNext(dec, &timeout);

    std::string result;
    if (reg) {
        DmtxMessage *msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
        if (msg) {
            result = std::string(
                reinterpret_cast<char *>(msg->output), msg->outputIdx);
            dmtxMessageDestroy(&msg);
        }
        dmtxRegionDestroy(&reg);
    }
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&dmtxImg);
    return result;
}

// Decode a data matrix from an 8-bit grayscale image. Returns empty string
// on failure. Tries Otsu-binarized and raw inputs across several shrink
// factors, since a single libdmtx pass often misses matrices that dominate
// the frame or have uneven illumination.
std::string readDataMatrix(const cv::Mat &gray8u) {
    cv::Mat binary;
    cv::threshold(gray8u, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    const std::vector<int> shrinkFactors = {2, 1, 4};
    const int perAttemptTimeoutMs = 1000;
    const std::vector<std::pair<const char *, const cv::Mat *>> inputs = {
        {"otsu", &binary}, {"raw", &gray8u}};
    for (const auto &[label, img] : inputs) {
        for (int shrink : shrinkFactors) {
            spdlog::debug("dmtx attempt: input={}, shrink={}", label, shrink);
            std::string result =
                tryDecodeDataMatrix(*img, shrink, perAttemptTimeoutMs);
            if (!result.empty()) {
                spdlog::info(
                    "Data matrix decoded (input={}, shrink={})", label, shrink);
                return result;
            }
        }
    }
    return "";
}

// Block until a frame with a receivedTime different from afterTime arrives.
FrameData waitForNextFrame(
    std::shared_ptr<LatestFrame> latestFrameHolder, uint64_t afterTime) {
    FrameData frame;
    do {
        frame = latestFrameHolder->getLatestFrameData();
        if (frame.receivedTime != afterTime)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (true);
    return frame;
}

// Per-axis sign between stage and arena coords: stage_pos = sign * arena_pos +
// offset. The arena is mounted face-down, which mirrors the X axis but leaves
// Y unchanged.
constexpr int kArenaXSign = -1;
constexpr int kArenaYSign = 1;

// Phase 1: live preview with crosshairs; wait for the user to centre the
// camera on the data matrix and press ENTER. On ENTER, returns true and the
// captured raw (un-reoriented) frame in rawFrameOut. On ESC, returns false so
// the caller can shut down and exit.
bool liveAlignmentPreview(
    const RecorderConfig &recorderConfig, cv::Mat &rawFrameOut) {
    cv::namedWindow("Behavior Camera", cv::WINDOW_NORMAL);
    // After reorientation the displayed dimensions are swapped relative to
    // sensor
    int roiWidth =
        recorderConfig.getParameter<int>("behavior_camera", "roi_width");
    int roiHeight =
        recorderConfig.getParameter<int>("behavior_camera", "roi_height");
    cv::resizeWindow("Behavior Camera", roiHeight / 2, roiWidth / 2);

    spdlog::info(
        "Live preview started. Move stages so camera is centered on the "
        "data matrix, then press ENTER.");

    while (true) {
        FrameData frameData =
            behaviorRecordingState->latestFrameHolder->getLatestFrameData();
        if (frameData.image.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::Mat oriented;
        reorientBehaviorImage(frameData.image, oriented);

        cv::Mat display;
        cv::cvtColor(oriented, display, cv::COLOR_GRAY2BGR);

        // Crosshairs at image center
        int cx = display.cols / 2;
        int cy = display.rows / 2;
        cv::Scalar red(0, 0, 255);
        cv::line(
            display, cv::Point(cx, 0), cv::Point(cx, display.rows - 1), red, 1);
        cv::line(
            display, cv::Point(0, cy), cv::Point(display.cols - 1, cy), red, 1);

        cv::putText(
            display,
            "Center camera on data matrix, then press ENTER",
            cv::Point(24, 66),
            cv::FONT_HERSHEY_SIMPLEX,
            1.5,
            red,
            3);

        // Downsample before imshow so the display pipeline isn't saturated by
        // full-resolution frames at the loop rate (caused a session lockup).
        cv::Mat displaySmall;
        cv::resize(display, displaySmall, {}, 0.5, 0.5);
        cv::imshow("Behavior Camera", displaySmall);
        int key = cv::waitKey(33);
        if (key == 13 || key == 10) // ENTER
        {
            // Capture the raw (un-reoriented) frame so dmtx sees the data
            // matrix in its un-mirrored orientation. The display has been
            // horizontally flipped to look natural to the user, but that
            // flip would mirror the data matrix and make it unreadable.
            rawFrameOut = frameData.image.clone();
            return true;
        }
        if (key == 27) // ESC
        {
            spdlog::info("ESC pressed. Exiting.");
            return false;
        }
    }
}

// Phases 2 & 3: decode the data matrix from the captured frame and verify its
// content against the checksum in metadata.yaml. On failure, logs the error and
// returns the distinguishing result (the caller is responsible for shutdown +
// throw).
enum class ChecksumResult { Success, NoDataMatrix, ChecksumMismatch };

ChecksumResult decodeAndVerifyChecksum(
    const cv::Mat &rawFrame, const YAML::Node &metadata) {
    spdlog::info("Decoding data matrix...");
    std::string dmContent = readDataMatrix(rawFrame);
    if (dmContent.empty()) {
        spdlog::error("No data matrix found in the captured frame.");
        return ChecksumResult::NoDataMatrix;
    }
    spdlog::info("Data matrix decoded: '{}'", dmContent);

    std::string expectedChecksum = metadata["checksum"].as<std::string>();
    if (dmContent != expectedChecksum) {
        spdlog::error(
            "Checksum mismatch: data matrix='{}', expected='{}'",
            dmContent,
            expectedChecksum);
        return ChecksumResult::ChecksumMismatch;
    }
    spdlog::info("Checksum verified: '{}'", dmContent);
    return ChecksumResult::Success;
}

// Phase 4: compute the offset mapping arena coords to stage coords, from the
// current stage position (centered on the data matrix) and the known data
// matrix center in arena coords.
void computeStageToArenaOffset(
    MotionControl &motionControl,
    const YAML::Node &metadata,
    double &offsetXOut,
    double &offsetYOut) {
    double currentX = motionControl.getPosition(X_AXIS);
    double currentY = motionControl.getPosition(Y_AXIS);
    spdlog::info(
        "Current stage position when centered on data matrix: ({:.4f}, {:.4f})",
        currentX,
        currentY);

    auto dmCenterVec =
        metadata["datamatrix_pos"]["center"].as<std::vector<double>>();
    double dmCenterX = dmCenterVec[0];
    double dmCenterY = dmCenterVec[1];
    offsetXOut = currentX - kArenaXSign * dmCenterX;
    offsetYOut = currentY - kArenaYSign * dmCenterY;
    spdlog::info(
        "Data matrix center in arena coords: ({:.4f}, {:.4f})",
        dmCenterX,
        dmCenterY);
    spdlog::info(
        "Axis signs (arena -> stage): x_sign={}, y_sign={}",
        kArenaXSign,
        kArenaYSign);
    spdlog::info(
        "Offset (arena -> stage): ({:.4f}, {:.4f})", offsetXOut, offsetYOut);
    spdlog::info(
        "Arena origin (0,0) maps to stage position ({:.4f}, {:.4f})",
        offsetXOut,
        offsetYOut);
}

// Live preview overlay during the apriltag visit; red dot = shutter.
void showApriltagStatus(int tagId, bool shutter) {
    FrameData fd =
        behaviorRecordingState->latestFrameHolder->getLatestFrameData();
    if (fd.image.empty())
        return;
    cv::Mat oriented, display;
    reorientBehaviorImage(fd.image, oriented);
    cv::cvtColor(oriented, display, cv::COLOR_GRAY2BGR);
    cv::Scalar red(0, 0, 255);
    int cx = display.cols / 2, cy = display.rows / 2;
    cv::line(display, {cx, 0}, {cx, display.rows - 1}, red, 1);
    cv::line(display, {0, cy}, {display.cols - 1, cy}, red, 1);
    cv::putText(
        display,
        fmt::format(
            "{} AprilTag #{}", shutter ? "Capturing" : "Moving to", tagId),
        {24, 66},
        cv::FONT_HERSHEY_SIMPLEX,
        1.5,
        red,
        3);
    if (shutter)
        cv::circle(display, {display.cols - 40, 40}, 20, red, -1);
    cv::Mat displaySmall;
    cv::resize(display, displaySmall, {}, 0.5, 0.5);
    cv::imshow("Behavior Camera", displaySmall);
    cv::waitKey(1);
}

// Phase 5: visit each AprilTag in id order, drop settling frames, acquire 10
// consecutive frames, and save images + CSV under <arenaDir>/mapping_scan.
// The onOutOfRange callback is invoked (then this rethrows) if a target stage
// position is outside the physical range of motion, so the caller can shut
// down before the exception propagates.
void scanAllApriltags(
    MotionControl &motionControl,
    const YAML::Node &metadata,
    double offsetX,
    double offsetY,
    double motionVelocity,
    int settlingFrames,
    const std::filesystem::path &arenaDir,
    const std::function<void()> &onOutOfRange) {
    YAML::Node apriltagPositions = metadata["apriltag_positions"];

    // Sort by integer key so we visit in a defined order
    std::map<int, YAML::Node> sortedTags;
    for (auto it = apriltagPositions.begin(); it != apriltagPositions.end();
         ++it)
        sortedTags[it->first.as<int>()] = it->second;

    std::filesystem::path scanDir = arenaDir / "mapping_scan";
    std::filesystem::create_directories(scanDir);
    std::filesystem::path csvPath = scanDir / "apriltag_stage_positions.csv";
    std::ofstream csvFile(csvPath.string());
    csvFile << "apriltag_id,image_id,stage_x_mm,stage_y_mm\n";

    std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 100};

    for (auto &[tagId, tagNode] : sortedTags) {
        auto tagCenter = tagNode["center"].as<std::vector<double>>();
        double targetX = kArenaXSign * tagCenter[0] + offsetX;
        double targetY = kArenaYSign * tagCenter[1] + offsetY;

        spdlog::info(
            "AprilTag {}: arena ({:.4f}, {:.4f}) -> stage ({:.4f}, {:.4f})",
            tagId,
            tagCenter[0],
            tagCenter[1],
            targetX,
            targetY);

        auto safeMoveAbsolute = [&](MotionAxis axis,
                                    const char *axisName,
                                    double target) {
            try {
                motionControl.moveAbsolute(axis, target, false, motionVelocity);
            } catch (const zaber::motion::exceptions::BadDataException &) {
                spdlog::error(
                    "Arena placed outside physical range of motion of linear "
                    "stages. Axis: {}, target: {:.4f} mm",
                    axisName,
                    target);
                onOutOfRange();
                throw std::runtime_error(
                    "Target stage position out of physical range");
            }
        };

        safeMoveAbsolute(X_AXIS, "X", targetX);
        safeMoveAbsolute(Y_AXIS, "Y", targetY);

        while (!motionControl.checkIfIdle(X_AXIS) ||
               !motionControl.checkIfIdle(Y_AXIS)) {
            showApriltagStatus(tagId, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Drop N settling frames (may have been exposed while stages were
        // still settling, or while mechanical vibration was decaying).
        for (int i = 0; i < settlingFrames; i++) {
            uint64_t lastTime =
                behaviorRecordingState->latestFrameHolder->getLatestFrameData()
                    .receivedTime;
            waitForNextFrame(
                behaviorRecordingState->latestFrameHolder, lastTime);
            showApriltagStatus(tagId, false);
        }
        spdlog::debug(
            "AprilTag {}: dropped {} settling frames", tagId, settlingFrames);

        // Acquire 10 consecutive frames
        for (int imgId = 0; imgId < 10; imgId++) {
            uint64_t lastTime =
                behaviorRecordingState->latestFrameHolder->getLatestFrameData()
                    .receivedTime;
            FrameData frameData = waitForNextFrame(
                behaviorRecordingState->latestFrameHolder, lastTime);

            cv::Mat image;
            reorientBehaviorImage(frameData.image, image);

            std::string filename =
                fmt::format("apriltag{}_img{}.jpg", tagId, imgId);
            cv::imwrite((scanDir / filename).string(), image, jpegParams);
            showApriltagStatus(tagId, true);

            double posX = motionControl.getPosition(X_AXIS);
            double posY = motionControl.getPosition(Y_AXIS);
            csvFile << tagId << "," << imgId << ","
                    << fmt::format("{:.6f}", posX) << ","
                    << fmt::format("{:.6f}", posY) << "\n";

            spdlog::debug(
                "Saved {} at stage ({:.4f}, {:.4f})", filename, posX, posY);
        }
        spdlog::info("AprilTag {} done: 10 frames saved", tagId);
    }

    csvFile.close();
    spdlog::info("Stage positions written to {}", csvPath.string());
}
} // namespace

void runArenaRegistrationScan(
    const std::filesystem::path &profileDir,
    const std::filesystem::path &arenaDir) {
    std::filesystem::path metadataPath = arenaDir / "metadata.yaml";

    if (!std::filesystem::exists(arenaDir)) {
        spdlog::error("Arena directory does not exist: {}", arenaDir.string());
        throw std::runtime_error(
            "Arena directory not found: " + arenaDir.string());
    }
    if (!std::filesystem::exists(metadataPath)) {
        spdlog::error(
            "Arena metadata file not found: {}", metadataPath.string());
        throw std::runtime_error(
            "metadata.yaml not found: " + metadataPath.string());
    }

    // Load recorder config
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info(
        "arenaRegistrationScan loading recorder configuration from {}",
        configPath.string());
    RecorderConfig recorderConfig(configPath);

    // Set up shared state and start behavior camera acquisition thread
    programState = std::make_shared<ProgramState>();
    auto programmedStop = std::make_shared<ProgrammedStop>();
    behaviorRecordingState = std::make_shared<BehaviorRecordingState>();
    behaviorRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();

    spdlog::info("Starting behavior camera acquisition thread");
    std::thread behaviorThread(
        behaviorImageAcquirer,
        recorderConfig,
        behaviorRecordingState,
        programState,
        programmedStop);

    // Wait for camera ready
    size_t retryCount = 0;
    while ((!behaviorRecordingState->behaviorCamera ||
            !behaviorRecordingState->behaviorCamera->isReady()) &&
           !programState->toQuit.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (++retryCount % 20 == 0)
            spdlog::warn("Waiting for behavior camera to initialize...");
    }
    // The behavior acquirer sets toQuit if the camera fails to open; abort cleanly
    // instead of spinning here forever. Triggering and motion are not set up yet,
    // so just stop and join the behavior acquirer before destroying its camera.
    if (programState->toQuit.load()) {
        spdlog::critical(
            "Behavior camera failed to initialize. Aborting arena registration "
            "scan.");
        if (behaviorRecordingState->behaviorCamera) {
            behaviorRecordingState->behaviorCamera->stop();
        }
        if (behaviorThread.joinable()) {
            behaviorThread.join();
        }
        behaviorRecordingState->behaviorCamera = nullptr;
        return;
    }
    spdlog::info("Behavior camera ready");

    // Start Arduino triggering. There is no muscle camera in the registration
    // scan, so leave muscle imaging off (the default): the behavior camera
    // free-runs. Were it on, the behavior camera would stall waiting for the
    // muscle camera's common-time signal and the live preview would freeze.
    arduinoCommunication =
        initializeTriggeringWithDefaultParams(
            recorderConfig,
            0,  // muscleNumLinesScanned (ignored since muscle cam not enabled)
            1,  // sync ratio (ignored since muscle cam not enabled)
            false); // muscleImagingOn

    // Set up motion control
    MotionControl motionControl(recorderConfig);
    double motionVelocity = recorderConfig.getParameter<double>(
        "motion_control", "default_velocity_mm_per_s");
    int settlingFrames = recorderConfig.getParameter<int>(
        "motion_control", "apriltag_mapping_settling_frames");

    auto shutdown = [&]() {
        programState->toQuit.store(true);
        // Interrupt the grab so the acquirer returns even if no frames are
        // arriving (waitForOneFrame() returns std::nullopt once stopped), JOIN
        // it, and only then destroy the camera -- the acquirer dereferences
        // behaviorRecordingState->behaviorCamera, so resetting it before the join
        // would be a use-after-free.
        if (behaviorRecordingState->behaviorCamera) {
            behaviorRecordingState->behaviorCamera->stop();
        }
        if (behaviorThread.joinable())
            behaviorThread.join();
        behaviorRecordingState->behaviorCamera = nullptr;
        // The new protocol has no "stop triggering" command; switch the blue
        // excitation light off, then close the link.
        arduinoCommunication->stopExcitation();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arduinoCommunication->stopCommunication();
    };

    // -------------------------------------------------------------------
    // Phase 1: Live preview with crosshairs; wait for user to press ENTER
    // -------------------------------------------------------------------
    cv::Mat rawFrame;
    if (!liveAlignmentPreview(recorderConfig, rawFrame)) {
        shutdown();
        return;
    }

    // -------------------------------------------------------------------
    // Phase 2 & 3: Decode data matrix and verify checksum against metadata
    // -------------------------------------------------------------------
    spdlog::info("Loading arena metadata from {}", metadataPath.string());
    YAML::Node metadata = YAML::LoadFile(metadataPath.string());
    ChecksumResult checksumResult = decodeAndVerifyChecksum(rawFrame, metadata);
    if (checksumResult != ChecksumResult::Success) {
        shutdown();
        if (checksumResult == ChecksumResult::NoDataMatrix) {
            throw std::runtime_error("No data matrix found");
        } else {
            throw std::runtime_error("Data matrix checksum mismatch");
        }
    }

    // -------------------------------------------------------------------
    // Phase 4: Calculate stage-to-arena offset
    // -------------------------------------------------------------------
    double offsetX, offsetY;
    computeStageToArenaOffset(motionControl, metadata, offsetX, offsetY);

    // -------------------------------------------------------------------
    // Phase 5: Visit each AprilTag, acquire 10 frames, save images + CSV
    // -------------------------------------------------------------------
    scanAllApriltags(
        motionControl,
        metadata,
        offsetX,
        offsetY,
        motionVelocity,
        settlingFrames,
        arenaDir,
        shutdown);

    // -------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------
    shutdown();
}

int main(int argc, char **argv) {
    std::signal(SIGINT, [](int) { quitProgram(); });

    // Parse CLI
    std::string profileDirStr = "~/Spotlight/default/";
    std::string arenaDirStr;
    spdlog::level::level_enum logLevel = spdlog::level::info;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            // clang-format off
            std::cout
                << "Usage: " << argv[0]
                << " -p PROFILE_DIR -a ARENA_DIR [OPTIONS]\n"
                << "  -p, --profile-dir PATH  Profile directory\n"
                << "  -a, --arena PATH        Arena directory (containing metadata.yaml)\n"
                << "  -v, --verbose           Debug-level logging\n"
                << "  --verbosity LEVEL       trace/debug/info/warn/error/critical/off\n";
            // clang-format on
            return 0;
        } else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc)
            profileDirStr = argv[++i];
        else if ((arg == "-a" || arg == "--arena") && i + 1 < argc)
            arenaDirStr = argv[++i];
        else if (arg == "-v" || arg == "--verbose")
            logLevel = spdlog::level::debug;
        else if (arg == "--verbosity" && i + 1 < argc) {
            std::string lvl = argv[++i];
            if (lvl == "trace")
                logLevel = spdlog::level::trace;
            else if (lvl == "debug")
                logLevel = spdlog::level::debug;
            else if (lvl == "warn")
                logLevel = spdlog::level::warn;
            else if (lvl == "error")
                logLevel = spdlog::level::err;
            else if (lvl == "critical")
                logLevel = spdlog::level::critical;
            else if (lvl == "off")
                logLevel = spdlog::level::off;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    if (arenaDirStr.empty()) {
        std::cerr << "Error: -a/--arena is required\n";
        return 1;
    }

    spdlog::set_level(logLevel);

    std::filesystem::path profileDir(expandPath(profileDirStr));
    std::filesystem::path arenaDir(expandPath(arenaDirStr));
    runArenaRegistrationScan(profileDir, arenaDir);
    spdlog::info("Arena registration complete");

    return 0;
}
