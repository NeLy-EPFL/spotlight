// Requires libdmtx-dev: sudo apt install libdmtx-dev
#include "registerArena.hpp"

#include <csignal>
#include <fstream>
#include <iostream>
#include <map>

#include <dmtx.h>
#include <yaml-cpp/yaml.h>

namespace
{
    std::shared_ptr<ProgramState> programState = nullptr;
    std::unique_ptr<ArduinoCommunication> arduinoCommunication = nullptr;
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState = nullptr;

    void quitProgram()
    {
        spdlog::info("SIGINT received. Initiating graceful shutdown");
        if (programState)
            programState->toQuit.store(true);
        if (arduinoCommunication)
        {
            arduinoCommunication->setBehaviorRecordingFPS(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            arduinoCommunication->stopCommunication();
        }
        std::exit(0);
    }

    // Decode a data matrix from an 8-bit grayscale image. Returns empty string on failure.
    std::string readDataMatrix(const cv::Mat &gray8u)
    {
        DmtxImage *dmtxImg = dmtxImageCreate(
            gray8u.data, gray8u.cols, gray8u.rows, DmtxPack8bppK);
        if (!dmtxImg)
            return "";

        DmtxDecode *dec = dmtxDecodeCreate(dmtxImg, 1);
        if (!dec)
        {
            dmtxImageDestroy(&dmtxImg);
            return "";
        }

        DmtxTime timeout = dmtxTimeAdd(dmtxTimeNow(), 5000);
        DmtxRegion *reg = dmtxRegionFindNext(dec, &timeout);

        std::string result;
        if (reg)
        {
            DmtxMessage *msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
            if (msg)
            {
                int len = msg->outputSize > 0 ? msg->outputSize - 1 : 0;
                result = std::string(reinterpret_cast<char *>(msg->output), len);
                dmtxMessageDestroy(&msg);
            }
            dmtxRegionDestroy(&reg);
        }
        dmtxDecodeDestroy(&dec);
        dmtxImageDestroy(&dmtxImg);
        return result;
    }

    // Block until a frame with a receivedTime different from afterTime arrives.
    FrameData waitForNextFrame(std::shared_ptr<LatestFrame> latestFrameHolder,
                               uint64_t afterTime)
    {
        FrameData frame;
        do
        {
            frame = latestFrameHolder->getLatestFrameData();
            if (frame.receivedTime != afterTime)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (true);
        return frame;
    }
}

void registerArena(std::filesystem::path profileDir, std::string arenaName)
{
    std::filesystem::path arenaDir = profileDir / "arenas" / arenaName;
    std::filesystem::path metadataPath = arenaDir / "metadata.yaml";

    if (!std::filesystem::exists(arenaDir))
    {
        spdlog::error("Arena directory does not exist: {}", arenaDir.string());
        throw std::runtime_error("Arena directory not found: " + arenaDir.string());
    }
    if (!std::filesystem::exists(metadataPath))
    {
        spdlog::error("Arena metadata file not found: {}", metadataPath.string());
        throw std::runtime_error("metadata.yaml not found: " + metadataPath.string());
    }

    // Load recorder config
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info("Loading recorder configuration from {}", configPath.string());
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
           !behaviorRecordingState->behaviorCamera->isReady())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (++retryCount % 20 == 0)
            spdlog::warn("Waiting for behavior camera to initialize...");
    }
    spdlog::info("Behavior camera ready");

    // Start Arduino triggering (behavior camera only; muscle params unused here)
    arduinoCommunication = initializeTriggeringWithDefaultParams(recorderConfig, 0, 1);

    // Set up motion control
    MotionControl motionControl(recorderConfig);
    double motionVelocity = recorderConfig.getParameter<double>(
        "motion_control", "default_velocity_mm_per_sec");

    // -------------------------------------------------------------------
    // Phase 1: Live preview with crosshairs; wait for user to press ENTER
    // -------------------------------------------------------------------
    cv::namedWindow("Behavior Camera", cv::WINDOW_NORMAL);
    // After reorientation the displayed dimensions are swapped relative to sensor
    int roiWidth  = recorderConfig.getParameter<int>("behavior_camera", "roi_width");
    int roiHeight = recorderConfig.getParameter<int>("behavior_camera", "roi_height");
    cv::resizeWindow("Behavior Camera", roiHeight / 2, roiWidth / 2);

    spdlog::info("Live preview started. Move stages so camera is centered on the "
                 "data matrix, then press ENTER.");

    cv::Mat rawFrame;
    while (true)
    {
        FrameData frameData = behaviorRecordingState->latestFrameHolder->getLatestFrameData();
        if (frameData.image.empty())
        {
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
        cv::Scalar green(0, 255, 0);
        cv::line(display, cv::Point(cx, 0), cv::Point(cx, display.rows - 1), green, 1);
        cv::line(display, cv::Point(0, cy), cv::Point(display.cols - 1, cy), green, 1);

        cv::putText(display,
                    "Center camera on data matrix, then press ENTER",
                    cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 255), 1);

        cv::imshow("Behavior Camera", display);
        int key = cv::waitKey(33);
        if (key == 13 || key == 10) // ENTER
        {
            rawFrame = frameData.image.clone();
            break;
        }
        if (key == 27) // ESC
        {
            spdlog::info("ESC pressed. Exiting.");
            programState->toQuit.store(true);
            behaviorRecordingState->behaviorCamera->stop();
            behaviorThread.join();
            arduinoCommunication->setBehaviorRecordingFPS(0);
            arduinoCommunication->stopCommunication();
            return;
        }
    }
    cv::destroyAllWindows();

    // -------------------------------------------------------------------
    // Phase 2: Decode data matrix
    // -------------------------------------------------------------------
    spdlog::info("Decoding data matrix...");
    std::string dmContent = readDataMatrix(rawFrame);
    if (dmContent.empty())
    {
        spdlog::error("No data matrix found in the captured frame.");
        programState->toQuit.store(true);
        behaviorRecordingState->behaviorCamera->stop();
        behaviorThread.join();
        arduinoCommunication->setBehaviorRecordingFPS(0);
        arduinoCommunication->stopCommunication();
        throw std::runtime_error("No data matrix found");
    }
    spdlog::info("Data matrix decoded: '{}'", dmContent);

    // -------------------------------------------------------------------
    // Phase 3: Load metadata and verify checksum
    // -------------------------------------------------------------------
    spdlog::info("Loading arena metadata from {}", metadataPath.string());
    YAML::Node metadata = YAML::LoadFile(metadataPath.string());
    std::string expectedChecksum = metadata["checksum"].as<std::string>();
    if (dmContent != expectedChecksum)
    {
        spdlog::error("Checksum mismatch: data matrix='{}', expected='{}'",
                      dmContent, expectedChecksum);
        programState->toQuit.store(true);
        behaviorRecordingState->behaviorCamera->stop();
        behaviorThread.join();
        arduinoCommunication->setBehaviorRecordingFPS(0);
        arduinoCommunication->stopCommunication();
        throw std::runtime_error("Data matrix checksum mismatch");
    }
    spdlog::info("Checksum verified: '{}'", dmContent);

    // -------------------------------------------------------------------
    // Phase 4: Calculate stage-to-arena offset
    // -------------------------------------------------------------------
    double currentX = motionControl.getPosition(X_AXIS);
    double currentY = motionControl.getPosition(Y_AXIS);
    spdlog::info("Current stage position when centered on data matrix: ({:.4f}, {:.4f})",
                 currentX, currentY);

    auto dmCenterVec = metadata["datamatrix_pos"]["center"].as<std::vector<double>>();
    double dmCenterX = dmCenterVec[0];
    double dmCenterY = dmCenterVec[1];
    // offset such that: stage_pos = arena_pos + offset
    double offsetX = currentX - dmCenterX;
    double offsetY = currentY - dmCenterY;
    spdlog::info("Data matrix center in arena coords: ({:.4f}, {:.4f})", dmCenterX, dmCenterY);
    spdlog::info("Offset (arena -> stage): ({:.4f}, {:.4f})", offsetX, offsetY);
    spdlog::info("Arena origin (0,0) maps to stage position ({:.4f}, {:.4f})", offsetX, offsetY);

    // -------------------------------------------------------------------
    // Phase 5: Visit each apriltag, acquire 10 frames, save images + CSV
    // -------------------------------------------------------------------
    YAML::Node apriltagPositions = metadata["apriltag_positions"];

    // Sort by integer key so we visit in a defined order
    std::map<int, YAML::Node> sortedTags;
    for (auto it = apriltagPositions.begin(); it != apriltagPositions.end(); ++it)
        sortedTags[it->first.as<int>()] = it->second;

    std::filesystem::path csvPath = arenaDir / "apriltag_stage_positions.csv";
    std::ofstream csvFile(csvPath.string());
    csvFile << "apriltag_id,image_id,stage_x_mm,stage_y_mm\n";

    std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 100};

    for (auto &[tagId, tagNode] : sortedTags)
    {
        auto tagCenter = tagNode["center"].as<std::vector<double>>();
        double targetX = tagCenter[0] + offsetX;
        double targetY = tagCenter[1] + offsetY;

        spdlog::info("Apriltag {}: arena ({:.4f}, {:.4f}) -> stage ({:.4f}, {:.4f})",
                     tagId, tagCenter[0], tagCenter[1], targetX, targetY);

        motionControl.moveAbsolute(X_AXIS, targetX, false, motionVelocity);
        motionControl.moveAbsolute(Y_AXIS, targetY, false, motionVelocity);

        while (!motionControl.checkIfIdle(X_AXIS) || !motionControl.checkIfIdle(Y_AXIS))
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Drop the first frame (may have been exposed while stages were still settling)
        uint64_t lastTime = behaviorRecordingState->latestFrameHolder
                                ->getLatestFrameData().receivedTime;
        waitForNextFrame(behaviorRecordingState->latestFrameHolder, lastTime);
        spdlog::debug("Apriltag {}: dropped initial frame", tagId);

        // Acquire 10 consecutive frames
        for (int imgId = 0; imgId < 10; imgId++)
        {
            lastTime = behaviorRecordingState->latestFrameHolder
                           ->getLatestFrameData().receivedTime;
            FrameData frameData =
                waitForNextFrame(behaviorRecordingState->latestFrameHolder, lastTime);

            cv::Mat image;
            reorientBehaviorImage(frameData.image, image);

            std::string filename = fmt::format("apriltag{}_img{}.jpg", tagId, imgId);
            cv::imwrite((arenaDir / filename).string(), image, jpegParams);

            double posX = motionControl.getPosition(X_AXIS);
            double posY = motionControl.getPosition(Y_AXIS);
            csvFile << tagId << "," << imgId << ","
                    << fmt::format("{:.6f}", posX) << ","
                    << fmt::format("{:.6f}", posY) << "\n";

            spdlog::debug("Saved {} at stage ({:.4f}, {:.4f})", filename, posX, posY);
        }
        spdlog::info("Apriltag {} done: 10 frames saved", tagId);
    }

    csvFile.close();
    spdlog::info("Stage positions written to {}", csvPath.string());

    // -------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------
    programState->toQuit.store(true);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (behaviorRecordingState->behaviorCamera)
    {
        behaviorRecordingState->behaviorCamera->stop();
        behaviorRecordingState->behaviorCamera = nullptr;
    }
    behaviorThread.join();

    arduinoCommunication->setBehaviorRecordingFPS(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arduinoCommunication->stopCommunication();
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, [](int) { quitProgram(); });

    // Parse CLI
    std::string profileDirStr = "~/Spotlight/default/";
    std::string arenaName;
    spdlog::level::level_enum logLevel = spdlog::level::info;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help")
        {
            std::cout << "Usage: " << argv[0]
                      << " -p PROFILE_DIR -a ARENA_NAME [OPTIONS]\n"
                      << "  -p, --profile-dir PATH  Profile directory\n"
                      << "  -a, --arena NAME        Arena name\n"
                      << "  -v, --verbose           Debug-level logging\n"
                      << "  --verbosity LEVEL       trace/debug/info/warn/error/critical/off\n";
            return 0;
        }
        else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc)
            profileDirStr = argv[++i];
        else if ((arg == "-a" || arg == "--arena") && i + 1 < argc)
            arenaName = argv[++i];
        else if (arg == "-v" || arg == "--verbose")
            logLevel = spdlog::level::debug;
        else if (arg == "--verbosity" && i + 1 < argc)
        {
            std::string lvl = argv[++i];
            if (lvl == "trace")    logLevel = spdlog::level::trace;
            else if (lvl == "debug")    logLevel = spdlog::level::debug;
            else if (lvl == "warn")     logLevel = spdlog::level::warn;
            else if (lvl == "error")    logLevel = spdlog::level::err;
            else if (lvl == "critical") logLevel = spdlog::level::critical;
            else if (lvl == "off")      logLevel = spdlog::level::off;
        }
        else
        {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    if (arenaName.empty())
    {
        std::cerr << "Error: -a/--arena is required\n";
        return 1;
    }

    spdlog::set_level(logLevel);

    std::filesystem::path profileDir(expandPath(profileDirStr));
    registerArena(profileDir, arenaName);
    spdlog::info("Arena registration complete");

    return 0;
}
