#include "alignCameras.hpp"

void alignCamera(RecorderConfig &recorderConfig)
{
    std::shared_ptr<ProgramState> programState =
        std::make_shared<ProgramState>();
    std::shared_ptr<ProgrammedStop> programmedRecordingStop =
        std::make_shared<ProgrammedStop>();

    // Set up behavior camera (JAI camera + Euresys frame grabber)
    spdlog::info("Starting behavior camera acquisition thread");
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState =
        std::make_shared<BehaviorRecordingState>();
    behaviorRecordingState->latestBehaviorFrameHolder =
        std::make_shared<LatestFrame>();
    std::thread behaviorImageAcquirerThread(
        behaviorImageAcquirer,
        recorderConfig,
        behaviorRecordingState,
        programState,
        programmedRecordingStop);

    // Load muscle camera full frame size
    unsigned int fullWidth = recorderConfig.getParameter<int>(
        "muscle_camera", "full_frame_width");
    unsigned int fullHeight = recorderConfig.getParameter<int>(
        "muscle_camera", "full_frame_height");

    // // Set up muscle camera (PCO camera)
    // spdlog::info("Setting up muscle camera");
    // std::shared_ptr<MuscleRecordingState> muscleRecordingState =
    //     std::make_shared<MuscleRecordingState>();
    // muscleRecordingState->latestBehaviorFrameHolder =
    //     std::make_shared<LatestFrame>();
    // std::thread muscleImageAcquirerThread(
    //     muscleImageAcquierer,
    //     fullWidth,
    //     fullHeight,
    //     0, // xOffset
    //     0, // yOffset
    //     recorderConfig,
    //     muscleRecordingState,
    //     programState,
    //     programmedRecordingStop);

    // Streaming loop
    spdlog::info("Starting streaming loop");
    cv::Mat behaviorImage;
    cv::Mat muscleImage;
    while (true)
    {
        behaviorImage = behaviorRecordingState
                            ->latestBehaviorFrameHolder
                            ->getLatestFrameData()
                            .image;
        // muscleImage = muscleRecordingState
        //                   ->latestBehaviorFrameHolder
        //                   ->getLatestFrameData()
        //                   .image;
        if (behaviorImage.empty() || muscleImage.empty())
        {
            spdlog::warn("One of the images is empty. Skipping display.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        cv::imshow("Behavior Camera", behaviorImage);
        cv::imshow("Muscle Camera", muscleImage);
        if (cv::waitKey(30) == 27)
        {
            break;
        }
    }

    // Stop the cameras
    spdlog::info("Stopping behavior camera acquisition thread");
    programState->toQuit.store(true);
    behaviorImageAcquirerThread.join();
    // muscleImageAcquirerThread.join();
    spdlog::info("Behavior camera acquisition thread stopped");
}

int main(int argc, char **argv)
{
    CLIOptions options = parseCLI(argc, argv);

    spdlog::set_level(options.logLevel);

    // Load recorder configuration
    std::filesystem::path profileDir =
        std::filesystem::path(expandPath(options.profileDir));
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info("Loading recorder configuration from {}", configPath.string());
    RecorderConfig recorderConfig(configPath);

    alignCamera(recorderConfig);
    spdlog::info("Calibration procedure complete");

    return 0;
}