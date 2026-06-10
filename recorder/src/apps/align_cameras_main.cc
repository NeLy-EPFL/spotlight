#include "recorder/apps/align_cameras.h"

#define DISPLAY_DOWNSAMPLE_FACTOR 3

namespace {

int fullMuscleImageWidth;
int fullMuscleImageHeight;
int muscleImageROIWidth;
int muscleImageROIHeight;
int userSelectedCenterXDisplay = -1;
int userSelectedCenterYDisplay = -1;
std::filesystem::path profileDir;
std::unique_ptr<ArduinoCommunication> arduinoCommunication = nullptr;

std::tuple<int, int> displayToCameraSensorCoords(int displayX, int displayY) {
    // Reverse the 90 degrees counterclockwise rotation and scale back up
    int sensorX =
        fullMuscleImageWidth - (displayY * DISPLAY_DOWNSAMPLE_FACTOR) - 1;
    int sensorY = displayX * DISPLAY_DOWNSAMPLE_FACTOR;
    return std::make_tuple(sensorX, sensorY);
}

std::tuple<int, int> cameraSensorToDisplayCoords(int sensorX, int sensorY) {
    // This is the opposite of displayToCameraSensorCoords
    int displayX = sensorY / DISPLAY_DOWNSAMPLE_FACTOR;
    int displayY =
        (fullMuscleImageWidth - sensorX - 1) / DISPLAY_DOWNSAMPLE_FACTOR;
    return std::make_tuple(displayX, displayY);
}

MuscleCameraROI
getROIFromDisplayCenter(int xCenterDisplay, int yCenterDisplay) {
    auto [muscleCameraCenterXSensor, muscleCameraCenterYSensor] =
        displayToCameraSensorCoords(xCenterDisplay, yCenterDisplay);
    int xOffset = muscleCameraCenterXSensor - (muscleImageROIWidth / 2);
    int yOffset = muscleCameraCenterYSensor - (muscleImageROIHeight / 2);
    // PCO ROI quantization: x offsets must be multiples of 32, y offsets
    // multiples of 8.
    int x0 = roundToNearestValidMuscleCamHorizontal(xOffset) + 1;
    int x1 = (x0 - 1) + muscleImageROIWidth;
    int y0 = roundToNearestValidMuscleCamVertical(yOffset) + 1;
    int y1 = (y0 - 1) + muscleImageROIHeight;

    MuscleCameraROI roi(x0, x1, y0, y1);
    return roi;
}

void drawMuscleImageROI(cv::Mat &image) {
    // Add red dot to muscle camera image
    if (userSelectedCenterXDisplay == -1 || userSelectedCenterYDisplay == -1) {
        // User didn't select a center point yet
        return;
    }

    // Draw user-selected center point
    cv::Scalar redColor(0, 0, 255);
    cv::Point pointIdeal(
        userSelectedCenterXDisplay, userSelectedCenterYDisplay);
    cv::circle(image, pointIdeal, 5, redColor, -1);

    // Figure out center point coords on the camera sensor
    MuscleCameraROI roi = getROIFromDisplayCenter(
        userSelectedCenterXDisplay, userSelectedCenterYDisplay);

    // Draw closest feasible center point
    auto [xCenterSensorActual, yCenterSensorActual] = roi.getCenterXY();
    auto [xCenterDisplayActual, yCenterDisplayActual] =
        cameraSensorToDisplayCoords(xCenterSensorActual, yCenterSensorActual);
    cv::Scalar blueColor(255, 0, 0);
    cv::Point pointActual(xCenterDisplayActual, yCenterDisplayActual);
    cv::circle(image, pointActual, 3, blueColor, -1);

    // Draw ROI rectangle
    auto [displayX0, displayY0] = cameraSensorToDisplayCoords(roi.x0, roi.y0);
    auto [displayX1, displayY1] = cameraSensorToDisplayCoords(roi.x1, roi.y1);
    cv::rectangle(
        image,
        cv::Point(displayX0, displayY0),
        cv::Point(displayX1, displayY1),
        blueColor,
        2); // thickness
}

void onMouse(int event, int x, int y, int flags, void *userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        userSelectedCenterXDisplay = x;
        userSelectedCenterYDisplay = y;

        // Convert display coordinates to camera sensor coordinates
        auto [sensorX, sensorY] = displayToCameraSensorCoords(x, y);
        spdlog::info(
            "Muscle camera center point set at (x={}, y={}) on "
            "the displayed image, which translates to (x={}, y={}) "
            "on the camera sensor.",
            x,
            y,
            sensorX,
            sensorY);
    }
}

void addCrossToBehaviorImage(cv::Mat &image) {
    // Add a cross to the middle of the behavior image to help with alignment
    cv::line(
        image,
        cv::Point(image.cols / 2, 0),
        cv::Point(image.cols / 2, image.rows),
        cv::Scalar(255, 255, 255),
        1);
    cv::line(
        image,
        cv::Point(0, image.rows / 2),
        cv::Point(image.cols, image.rows / 2),
        cv::Scalar(255, 255, 255),
        1);
}

bool trySaveSelectedROI(const std::filesystem::path &profileDir) {
    // Validate the user-selected ROI center and, if it is in bounds, write the
    // resulting ROI to <profileDir>/muscle_camera_roi.yaml. Returns true when an
    // ROI was successfully saved (caller should break out of the streaming
    // loop), false otherwise (caller should continue).
    if (userSelectedCenterXDisplay == -1 || userSelectedCenterYDisplay == -1) {
        spdlog::error(
            "ROI center not set yet. Please select a center point on "
            "the muscle camera image before saving the ROI.");
        return false;
    }

    spdlog::info(
        "User selected muscle camera center point at (x={}, y={})",
        userSelectedCenterXDisplay,
        userSelectedCenterYDisplay);
    MuscleCameraROI roi = getROIFromDisplayCenter(
        userSelectedCenterXDisplay, userSelectedCenterYDisplay);
    if (roi.x0 <= 0 || roi.y0 <= 0 || roi.x1 > fullMuscleImageWidth ||
        roi.y1 > fullMuscleImageHeight) {
        spdlog::error("Selected ROI out of bound. Please try again.");
        return false;
    }

    std::filesystem::path roiFilePath = profileDir / "muscle_camera_roi.yaml";
    spdlog::info(
        "Saving muscle camera ROI (x0={}, x1={}, y0={}, y1={}) to {}",
        roi.x0,
        roi.x1,
        roi.y0,
        roi.y1,
        roiFilePath.string());
    roi.toFile(roiFilePath);

    return true;
}

void setupDisplayWindows(const RecorderConfig &recorderConfig) {
    // Figure out window display size (note width/height are swapped because
    // images are roatated)
    int behaviorImageDisplayWidth =
        recorderConfig.getParameter<int>("behavior_camera", "roi_height") /
        DISPLAY_DOWNSAMPLE_FACTOR;
    int behaviorImageDisplayHeight =
        recorderConfig.getParameter<int>("behavior_camera", "roi_width") /
        DISPLAY_DOWNSAMPLE_FACTOR;
    int muscleImageDisplayWidth =
        fullMuscleImageHeight / DISPLAY_DOWNSAMPLE_FACTOR;
    int muscleImageDisplayHeight =
        fullMuscleImageWidth / DISPLAY_DOWNSAMPLE_FACTOR;

    // Create named windows
    cv::namedWindow("Behavior Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow(
        "Behavior Camera",
        behaviorImageDisplayWidth,
        behaviorImageDisplayHeight);

    cv::namedWindow("Muscle Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow(
        "Muscle Camera", muscleImageDisplayWidth, muscleImageDisplayHeight);

    // Add callback for muscle camera window
    cv::setMouseCallback("Muscle Camera", onMouse, nullptr);
}
} // namespace

void alignCamera(const std::filesystem::path &profileDir) {
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info(
        "alignCameras loading recorder configuration from {}",
        configPath.string());
    RecorderConfig recorderConfig(configPath);

    // Load muscle camera full frame size
    fullMuscleImageWidth =
        recorderConfig.getParameter<int>("muscle_camera", "full_frame_width");
    fullMuscleImageHeight =
        recorderConfig.getParameter<int>("muscle_camera", "full_frame_height");
    muscleImageROIWidth =
        recorderConfig.getParameter<int>("muscle_camera", "roi_width");
    muscleImageROIHeight =
        recorderConfig.getParameter<int>("muscle_camera", "roi_height");

    // Load the normalization window for displaying the 16-bit muscle image
    // during camera alignment (separate from the main GUI's histogram defaults).
    int muscleDisplayVmin = recorderConfig.getParameter<int>(
        "muscle_camera", "align_cameras_display_vmin");
    int muscleDisplayVmax = recorderConfig.getParameter<int>(
        "muscle_camera", "align_cameras_display_vmax");

    // Set up shared recording states
    std::shared_ptr<ProgramState> programState =
        std::make_shared<ProgramState>();
    std::shared_ptr<ProgrammedStop> programmedRecordingStop =
        std::make_shared<ProgrammedStop>();
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState =
        std::make_shared<BehaviorRecordingState>();
    std::shared_ptr<MuscleRecordingState> muscleRecordingState =
        std::make_shared<MuscleRecordingState>();

    // Set up cameras acquisition threads
    spdlog::info("Starting behavior camera acquisition thread");
    behaviorRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();
    std::thread behaviorImageAcquirerThread(
        behaviorImageAcquirer,
        recorderConfig,
        behaviorRecordingState,
        programState,
        programmedRecordingStop);
    spdlog::info("Behavior camera acquisition thread started");

    // Declared now (assigned once the behavior camera is ready) so the shutdown
    // lambda below can join it on an early abort even before it is started.
    std::thread muscleImageAcquirerThread;

    // Single teardown path, used both on a normal exit and on an early abort when
    // a camera fails to initialize. Signals shutdown, interrupts both camera grabs
    // so the acquirers return even if no frames are arriving (waitForOneFrame()
    // returns std::nullopt once stopped), JOINs both acquirers before destroying
    // either camera (each in-process camera is touched only by its acquirer thread
    // and must outlive it), then switches the excitation light off and closes the
    // Arduino link if triggering was ever started.
    auto shutdown = [&]() {
        programState->toQuit.store(true);
        if (behaviorRecordingState->behaviorCamera) {
            behaviorRecordingState->behaviorCamera->stop();
        }
        if (muscleRecordingState->muscleCamera) {
            muscleRecordingState->muscleCamera->stop();
        }
        if (behaviorImageAcquirerThread.joinable()) {
            behaviorImageAcquirerThread.join();
        }
        if (muscleImageAcquirerThread.joinable()) {
            muscleImageAcquirerThread.join();
        }
        behaviorRecordingState->behaviorCamera = nullptr;
        muscleRecordingState->muscleCamera = nullptr;
        if (arduinoCommunication) {
            spdlog::info("Switching off excitation and closing Arduino link");
            arduinoCommunication->stopExcitation();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            arduinoCommunication->stopCommunication();
        }
    };

    // Initialize the behavior (Euresys) camera fully before starting the muscle
    // (PCO) camera thread: the two camera SDKs must not run their library init /
    // device discovery concurrently in this process (it was observed to hang the
    // Euresys GenTL discovery). See the same guard in run_spotlight_main.cc.
    spdlog::info(
        "Waiting for behavior camera to initialize before starting the muscle "
        "camera...");
    while ((!behaviorRecordingState->behaviorCamera ||
            !behaviorRecordingState->behaviorCamera->isReady()) &&
           !programState->toQuit.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // The behavior acquirer sets toQuit if the camera fails to open; abort cleanly
    // instead of spinning here forever.
    if (programState->toQuit.load()) {
        spdlog::critical(
            "Behavior camera failed to initialize. Aborting align-cameras.");
        shutdown();
        return;
    }
    spdlog::info("Behavior camera initialized; starting muscle camera.");

    spdlog::info("Setting up muscle camera acquisition thread");
    muscleRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();
    muscleImageAcquirerThread = std::thread(
        muscleImageAcquirer,
        fullMuscleImageWidth,
        fullMuscleImageHeight,
        0, // xOffset
        0, // yOffset
        recorderConfig,
        muscleRecordingState,
        programState,
        programmedRecordingStop);
    spdlog::info("Muscle camera acquisition thread started");

    // Start Arduino triggering interface set default triggering parameters
    size_t retryCount = 0;
    while (!muscleRecordingState->muscleCamera &&
           !programState->toQuit.load()) {
        spdlog::debug("Waiting for muscle camera to be ready");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retryCount++;
        if (retryCount % 10 == 0) {
            spdlog::warn("Muscle camera is not initialized.");
        }
    }
    // The muscle acquirer sets toQuit if the camera fails to open; abort cleanly
    // instead of spinning here forever (and before dereferencing muscleCamera).
    if (programState->toQuit.load()) {
        spdlog::critical(
            "Muscle camera failed to initialize. Aborting align-cameras.");
        shutdown();
        return;
    }
    int muscleNumLinesScanned =
        muscleRecordingState->muscleCamera->getNumLinesScanned();
    arduinoCommunication = initializeTriggeringWithDefaultParams(
        recorderConfig,
        muscleNumLinesScanned,
        1,     // sync ratio
        true); // muscleImagingOn

    // Set up display windows
    setupDisplayWindows(recorderConfig);

    // Streaming loop
    spdlog::info("Starting streaming loop");
    cv::Mat behaviorImage;
    cv::Mat muscleImage;
    cv::Mat behaviorImageDisplay;
    cv::Mat muscleImageDisplay;
    while (true) {
        // Fetch latest images from both cameras
        behaviorImage =
            behaviorRecordingState->latestFrameHolder->getLatestFrameData()
                .image;
        muscleImage =
            muscleRecordingState->latestFrameHolder->getLatestFrameData().image;

        // Skip display if images are empty
        if (behaviorImage.empty()) {
            spdlog::warn(
                "Behavior image is empty. Skipping display. This is normal "
                "if it only happens a few times at the beginning of the "
                "program while the camera initializes.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        if (muscleImage.empty()) {
            spdlog::warn(
                "Muscle image is empty. Skipping display. This is normal "
                "if it only happens a few times at the beginning of the "
                "program while the camera initializes.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        // Resize images for display
        cv::Size targetSize;
        targetSize = cv::Size(
            behaviorImage.cols / DISPLAY_DOWNSAMPLE_FACTOR,
            behaviorImage.rows / DISPLAY_DOWNSAMPLE_FACTOR);
        cv::resize(behaviorImage, behaviorImageDisplay, targetSize);
        reorientBehaviorImage(behaviorImageDisplay, behaviorImageDisplay);
        // Add a cross to the middle of the behavior image to help with
        // alignment
        addCrossToBehaviorImage(behaviorImageDisplay);

        convert16BitTo8Bit(
            muscleImage, muscleImage, muscleDisplayVmin, muscleDisplayVmax);
        targetSize = cv::Size(
            muscleImage.cols / DISPLAY_DOWNSAMPLE_FACTOR,
            muscleImage.rows / DISPLAY_DOWNSAMPLE_FACTOR);
        cv::resize(muscleImage, muscleImageDisplay, targetSize);
        reorientMuscleImage(muscleImageDisplay, muscleImageDisplay);
        cv::cvtColor(
            muscleImageDisplay, muscleImageDisplay, cv::COLOR_GRAY2BGR);

        // Draw markers on the displayed image to indicate ROI
        drawMuscleImageROI(muscleImageDisplay);

        // Display images
        cv::imshow("Behavior Camera", behaviorImageDisplay);
        cv::imshow("Muscle Camera", muscleImageDisplay);
        int pressedKey = cv::waitKey(1);
        if (pressedKey == 27) {
            // ESC key pressed
            break;
        }
        if (pressedKey == 13) {
            // Enter key pressed
            if (trySaveSelectedROI(profileDir)) {
                break;
            }
            continue;
        }
    }

    // Stop the cameras and triggering via the single teardown path defined above
    // (joins both acquirers before destroying the cameras, then switches off
    // excitation and closes the Arduino link).
    spdlog::info("Stopping camera acquisition threads");
    shutdown();
    spdlog::info("Camera acquisition threads stopped");
}

int main(int argc, char **argv) {
    CLIOptions options = parseCLI(argc, argv);

    spdlog::set_level(options.logLevel);

    // Load recorder configuration
    profileDir = std::filesystem::path(expandPath(options.profileDir));

    alignCamera(profileDir);
    spdlog::info("Calibration procedure complete");

    return 0;
}