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
#include "runArenaRegistrationScan.hpp"

#include <csignal>
#include <fstream>
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
        arduinoCommunication->setBehaviorRecordingFPS(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arduinoCommunication->stopCommunication();
    }
    std::exit(0);
}

// Single decode attempt at a given shrink factor.
std::string tryDecodeDataMatrix(const cv::Mat &gray8u, int shrink, int timeoutMs) {
    DmtxImage *dmtxImg = dmtxImageCreate(gray8u.data, gray8u.cols, gray8u.rows, DmtxPack8bppK);
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
            result = std::string(reinterpret_cast<char *>(msg->output), msg->outputIdx);
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
            std::string result = tryDecodeDataMatrix(*img, shrink, perAttemptTimeoutMs);
            if (!result.empty()) {
                spdlog::info("Data matrix decoded (input={}, shrink={})", label, shrink);
                return result;
            }
        }
    }
    return "";
}

// Block until a frame with a receivedTime different from afterTime arrives.
FrameData waitForNextFrame(std::shared_ptr<LatestFrame> latestFrameHolder, uint64_t afterTime) {
    FrameData frame;
    do {
        frame = latestFrameHolder->getLatestFrameData();
        if (frame.receivedTime != afterTime)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (true);
    return frame;
}
} // namespace

void runArenaRegistrationScan(std::filesystem::path profileDir, std::filesystem::path arenaDir) {
    std::filesystem::path metadataPath = arenaDir / "metadata.yaml";

    if (!std::filesystem::exists(arenaDir)) {
        spdlog::error("Arena directory does not exist: {}", arenaDir.string());
        throw std::runtime_error("Arena directory not found: " + arenaDir.string());
    }
    if (!std::filesystem::exists(metadataPath)) {
        spdlog::error("Arena metadata file not found: {}", metadataPath.string());
        throw std::runtime_error("metadata.yaml not found: " + metadataPath.string());
    }

    // Load recorder config
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info(
        "arenaRegistrationScan loading recorder configuration from {}", configPath.string());
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
    while (!behaviorRecordingState->behaviorCamera ||
           !behaviorRecordingState->behaviorCamera->isReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (++retryCount % 20 == 0)
            spdlog::warn("Waiting for behavior camera to initialize...");
    }
    spdlog::info("Behavior camera ready");

    // Start Arduino triggering (behavior camera only; muscle params unused here)
    arduinoCommunication = initializeTriggeringWithDefaultParams(recorderConfig, 0, 1);

    // Set up motion control
    MotionControl motionControl(recorderConfig);
    double motionVelocity =
        recorderConfig.getParameter<double>("motion_control", "default_velocity_mm_per_sec");
    int settlingFrames =
        recorderConfig.getParameter<int>("motion_control", "apriltag_mapping_settling_frames");

    auto shutdown = [&]() {
        programState->toQuit.store(true);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (behaviorRecordingState->behaviorCamera) {
            behaviorRecordingState->behaviorCamera->stop();
            behaviorRecordingState->behaviorCamera = nullptr;
        }
        if (behaviorThread.joinable())
            behaviorThread.join();
        arduinoCommunication->setBehaviorRecordingFPS(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arduinoCommunication->stopCommunication();
    };

    // -------------------------------------------------------------------
    // Phase 1: Live preview with crosshairs; wait for user to press ENTER
    // -------------------------------------------------------------------
    cv::namedWindow("Behavior Camera", cv::WINDOW_NORMAL);
    // After reorientation the displayed dimensions are swapped relative to sensor
    int roiWidth = recorderConfig.getParameter<int>("behavior_camera", "roi_width");
    int roiHeight = recorderConfig.getParameter<int>("behavior_camera", "roi_height");
    cv::resizeWindow("Behavior Camera", roiHeight / 2, roiWidth / 2);

    spdlog::info("Live preview started. Move stages so camera is centered on the "
                 "data matrix, then press ENTER.");

    cv::Mat rawFrame;
    while (true) {
        FrameData frameData = behaviorRecordingState->latestFrameHolder->getLatestFrameData();
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
        cv::line(display, cv::Point(cx, 0), cv::Point(cx, display.rows - 1), red, 1);
        cv::line(display, cv::Point(0, cy), cv::Point(display.cols - 1, cy), red, 1);

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
            rawFrame = frameData.image.clone();
            break;
        }
        if (key == 27) // ESC
        {
            spdlog::info("ESC pressed. Exiting.");
            shutdown();
            return;
        }
    }

    // -------------------------------------------------------------------
    // Phase 2: Decode data matrix
    // -------------------------------------------------------------------
    spdlog::info("Decoding data matrix...");
    std::string dmContent = readDataMatrix(rawFrame);
    if (dmContent.empty()) {
        spdlog::error("No data matrix found in the captured frame.");
        shutdown();
        throw std::runtime_error("No data matrix found");
    }
    spdlog::info("Data matrix decoded: '{}'", dmContent);

    // -------------------------------------------------------------------
    // Phase 3: Load metadata and verify checksum
    // -------------------------------------------------------------------
    spdlog::info("Loading arena metadata from {}", metadataPath.string());
    YAML::Node metadata = YAML::LoadFile(metadataPath.string());
    std::string expectedChecksum = metadata["checksum"].as<std::string>();
    if (dmContent != expectedChecksum) {
        spdlog::error(
            "Checksum mismatch: data matrix='{}', expected='{}'", dmContent, expectedChecksum);
        shutdown();
        throw std::runtime_error("Data matrix checksum mismatch");
    }
    spdlog::info("Checksum verified: '{}'", dmContent);

    // -------------------------------------------------------------------
    // Phase 4: Calculate stage-to-arena offset
    // -------------------------------------------------------------------
    double currentX = motionControl.getPosition(X_AXIS);
    double currentY = motionControl.getPosition(Y_AXIS);
    spdlog::info(
        "Current stage position when centered on data matrix: ({:.4f}, {:.4f})",
        currentX,
        currentY);

    auto dmCenterVec = metadata["datamatrix_pos"]["center"].as<std::vector<double>>();
    double dmCenterX = dmCenterVec[0];
    double dmCenterY = dmCenterVec[1];
    // Per-axis sign between stage and arena coords: stage_pos = sign * arena_pos
    // + offset. The arena is mounted face-down, which mirrors the X axis but
    // leaves Y unchanged.
    const int xSign = -1;
    const int ySign = 1;
    double offsetX = currentX - xSign * dmCenterX;
    double offsetY = currentY - ySign * dmCenterY;
    spdlog::info("Data matrix center in arena coords: ({:.4f}, {:.4f})", dmCenterX, dmCenterY);
    spdlog::info("Axis signs (arena -> stage): x_sign={}, y_sign={}", xSign, ySign);
    spdlog::info("Offset (arena -> stage): ({:.4f}, {:.4f})", offsetX, offsetY);
    spdlog::info("Arena origin (0,0) maps to stage position ({:.4f}, {:.4f})", offsetX, offsetY);

    // -------------------------------------------------------------------
    // Phase 5: Visit each AprilTag, acquire 10 frames, save images + CSV
    // -------------------------------------------------------------------
    YAML::Node apriltagPositions = metadata["apriltag_positions"];

    // Sort by integer key so we visit in a defined order
    std::map<int, YAML::Node> sortedTags;
    for (auto it = apriltagPositions.begin(); it != apriltagPositions.end(); ++it)
        sortedTags[it->first.as<int>()] = it->second;

    std::filesystem::path scanDir = arenaDir / "mapping_scan";
    std::filesystem::create_directories(scanDir);
    std::filesystem::path csvPath = scanDir / "apriltag_stage_positions.csv";
    std::ofstream csvFile(csvPath.string());
    csvFile << "apriltag_id,image_id,stage_x_mm,stage_y_mm\n";

    std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 100};

    // Live preview overlay during the apriltag visit; red dot = shutter.
    auto showStatus = [&](int tagId, bool shutter) {
        FrameData fd = behaviorRecordingState->latestFrameHolder->getLatestFrameData();
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
            fmt::format("{} AprilTag #{}", shutter ? "Capturing" : "Moving to", tagId),
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
    };

    for (auto &[tagId, tagNode] : sortedTags) {
        auto tagCenter = tagNode["center"].as<std::vector<double>>();
        double targetX = xSign * tagCenter[0] + offsetX;
        double targetY = ySign * tagCenter[1] + offsetY;

        spdlog::info(
            "AprilTag {}: arena ({:.4f}, {:.4f}) -> stage ({:.4f}, {:.4f})",
            tagId,
            tagCenter[0],
            tagCenter[1],
            targetX,
            targetY);

        auto safeMoveAbsolute = [&](MotionAxis axis, const char *axisName, double target) {
            try {
                motionControl.moveAbsolute(axis, target, false, motionVelocity);
            } catch (const zaber::motion::exceptions::BadDataException &) {
                spdlog::error(
                    "Arena placed outside physical range of motion of linear "
                    "stages. Axis: {}, target: {:.4f} mm",
                    axisName,
                    target);
                shutdown();
                throw std::runtime_error("Target stage position out of physical range");
            }
        };

        safeMoveAbsolute(X_AXIS, "X", targetX);
        safeMoveAbsolute(Y_AXIS, "Y", targetY);

        while (!motionControl.checkIfIdle(X_AXIS) || !motionControl.checkIfIdle(Y_AXIS)) {
            showStatus(tagId, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Drop N settling frames (may have been exposed while stages were
        // still settling, or while mechanical vibration was decaying).
        for (int i = 0; i < settlingFrames; i++) {
            uint64_t lastTime =
                behaviorRecordingState->latestFrameHolder->getLatestFrameData().receivedTime;
            waitForNextFrame(behaviorRecordingState->latestFrameHolder, lastTime);
            showStatus(tagId, false);
        }
        spdlog::debug("AprilTag {}: dropped {} settling frames", tagId, settlingFrames);

        // Acquire 10 consecutive frames
        for (int imgId = 0; imgId < 10; imgId++) {
            uint64_t lastTime =
                behaviorRecordingState->latestFrameHolder->getLatestFrameData().receivedTime;
            FrameData frameData =
                waitForNextFrame(behaviorRecordingState->latestFrameHolder, lastTime);

            cv::Mat image;
            reorientBehaviorImage(frameData.image, image);

            std::string filename = fmt::format("apriltag{}_img{}.jpg", tagId, imgId);
            cv::imwrite((scanDir / filename).string(), image, jpegParams);
            showStatus(tagId, true);

            double posX = motionControl.getPosition(X_AXIS);
            double posY = motionControl.getPosition(Y_AXIS);
            csvFile << tagId << "," << imgId << "," << fmt::format("{:.6f}", posX) << ","
                    << fmt::format("{:.6f}", posY) << "\n";

            spdlog::debug("Saved {} at stage ({:.4f}, {:.4f})", filename, posX, posY);
        }
        spdlog::info("AprilTag {} done: 10 frames saved", tagId);
    }

    csvFile.close();
    spdlog::info("Stage positions written to {}", csvPath.string());

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
            std::cout << "Usage: " << argv[0] << " -p PROFILE_DIR -a ARENA_DIR [OPTIONS]\n"
                      << "  -p, --profile-dir PATH  Profile directory\n"
                      << "  -a, --arena PATH        Arena directory (containing metadata.yaml)\n"
                      << "  -v, --verbose           Debug-level logging\n"
                      << "  --verbosity LEVEL       trace/debug/info/warn/error/critical/off\n";
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
