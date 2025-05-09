#include "trackingControl.hpp"

// Shared global variables and sysnchronization primitives
namespace
{
    std::mutex requestMutex;
    std::condition_variable requestCondVar;
    std::mutex responseMutex;
    std::condition_variable responseCondVar;
    std::queue<MotionStageRequest> requestQueue;
    std::map<int, MotionStageResponse> responseMap;

    std::ofstream initializeMotionStageLogFile(std::string saveDirectory)
    {
        std::filesystem::path filename =
            fs::path(saveDirectory) / "stage_position" / "stage_position.csv";
        std::ofstream logFile((filename).string(), std::ios_base::app);
        if (!logFile.is_open())
        {
            spdlog::error("Failed to open motion stage log file: {}",
                          filename.string());
        }
        else
        {
            spdlog::info("Opened motion stage log file: {}",
                         filename.string());
        }
        logFile << "timestamp_us,x_pos_mm,y_pos_mm\n";
        return logFile;
    }

    double calculateDistance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

    int imageBinarizeThreshold;
}

void motionControlRequestHandler(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<TrackingControlState> trackingControlState,
    std::shared_ptr<ProgramState> programState)
{
    MotionControl motionControl(recorderConfig);
    trackingControlState->motionControlHandlerReady.store(true);

    while (!programState->toQuit.load())
    {
        MotionStageRequest myRequest;
        // Wait for a request
        {
            std::unique_lock<std::mutex> lock(requestMutex);
            requestCondVar.wait(lock, [programState]
                                { return !requestQueue.empty() ||
                                         programState->toQuit.load(); });

            // If there's still work to do, finish it even if told to stop
            if (!requestQueue.empty())
            {
                myRequest = requestQueue.front();
                requestQueue.pop();
            }
            else
            {
                // Only way to reach here is if toQuit is true
                assert(programState->toQuit.load());
                spdlog::info(
                    "Motion stage request handler thread "
                    "is breaking out of loop.");
                break;
            }
        }

        MotionStageResponse myResponse;
        if (myRequest.requestType == GET_CURRENT_POSITION)
        {
            MotionStagePosition currentPosition = {
                motionControl.getPosition(X_AXIS),
                motionControl.getPosition(Y_AXIS),
                ABSOLUTE};
            myResponse.position = currentPosition;
        }
        else if (myRequest.requestType == SET_TARGET_POSITION)
        {
            bool waitForCompletion = false;
            if (myRequest.position.positionType == ABSOLUTE)
            {
                motionControl.moveAbsolute(X_AXIS,
                                           myRequest.position.xPosMm,
                                           waitForCompletion,
                                           myRequest.velocity);
                motionControl.moveAbsolute(Y_AXIS,
                                           myRequest.position.yPosMm,
                                           waitForCompletion,
                                           myRequest.velocity);
            }
            else
            {
                motionControl.moveRelative(X_AXIS,
                                           myRequest.position.xPosMm,
                                           waitForCompletion,
                                           myRequest.velocity);
                motionControl.moveRelative(Y_AXIS,
                                           myRequest.position.yPosMm,
                                           waitForCompletion,
                                           myRequest.velocity);
            }
            myResponse.setSuccess = true;
        }
        else if (myRequest.requestType == WAIT_UNTIL_IDLE)
        {
            motionControl.waitUntilIdle(X_AXIS);
            motionControl.waitUntilIdle(Y_AXIS);
            myResponse.isIdle = true;
        }
        else if (myRequest.requestType == CHECK_IF_IDLE)
        {
            myResponse.isIdle = motionControl.checkIfIdle(X_AXIS) &&
                                motionControl.checkIfIdle(Y_AXIS);
        }
        else if (myRequest.requestType == START_HOMING)
        {
            bool waitForCompletion = false;
            motionControl.home(X_AXIS, waitForCompletion);
            motionControl.home(Y_AXIS, waitForCompletion);
            myResponse.setSuccess = true;
        }
        else
        {
            spdlog::critical(
                "Motion stage request handler thread received unknown "
                "request type: {}",
                myRequest.requestType);
            throw std::runtime_error(
                "Motion stage request handler thread received unknown "
                "request type.");
        }

        // Send response back
        {
            std::lock_guard<std::mutex> lock(responseMutex);
            responseMap[myRequest.clientIdHash] = myResponse;
        }
        responseCondVar.notify_all();
    }
    spdlog::info("Motion stage request handler thread stopped.");
}

void trackingController(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<TrackingControlState> trackingControlState,
    CalibrationParams &behaviorCamCalibrationParams,
    std::shared_ptr<ProgramState> programState)
{
    while (!trackingControlState->motionControlHandlerReady.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    int trackingUpdateFrequency = recorderConfig.getParameter<int>(
        "tracking", "update_frequency_hz");
    uint64_t updateIntervalMicrosecs = 1e6 / trackingUpdateFrequency;

    float trackingDistanceThresholdMm = recorderConfig.getParameter<float>(
        "tracking", "distance_threshold_for_moving_mm");

    float defaultVelocity = recorderConfig.getParameter<float>(
        "motion_control", "default_velocity_mm_per_sec");

    imageBinarizeThreshold = recorderConfig.getParameter<int>(
        "tracking", "image_binarize_threshold");

    while (!programState->toQuit.load())
    {
        uint64_t startTime = getCurrentTimeMicroseconds();

        if (!trackingControlState->trackingOn.load())
        {
            // Nothing to do here
        }
        else if (!trackingControlState->shouldOverrideTracking.load())
        {
            cv::Mat myBehaviorImage = behaviorRecordingState
                                          ->latestFrameHolder
                                          ->getLatestFrameData()
                                          .image;
            reorientBehaviorImage(myBehaviorImage, myBehaviorImage);

            MotionStagePosition myMotionStagePosition;
            {
                std::lock_guard<std::mutex> lock(
                    trackingControlState->latestMotionStagePositionMutex);
                myMotionStagePosition =
                    trackingControlState->latestMotionStagePosition;
            }

            bool isFound = false;
            double physicalPosX = 0;
            double physicalPosY = 0;
            if (behaviorRecordingState->behaviorCamera &&
                behaviorRecordingState->behaviorCamera->isReady())
            {
                std::tie(isFound, physicalPosX, physicalPosY) =
                    calculateFlyPositionAbsoluteMm(myBehaviorImage,
                                                   myMotionStagePosition,
                                                   behaviorCamCalibrationParams,
                                                   recorderConfig);
            }

            if (isFound)
            {
                auto [currentPhysicalPosX, currentPhysicalPosY] =
                    behaviorCamCalibrationParams
                        .stagePosAndPixelPosToPhysicalPos(
                            myMotionStagePosition.xPosMm,
                            myMotionStagePosition.yPosMm,
                            myBehaviorImage.rows / 2,
                            myBehaviorImage.cols / 2);

                double distanceToTarget = calculateDistance(
                    physicalPosX,
                    physicalPosY,
                    currentPhysicalPosX,
                    currentPhysicalPosY);

                if (distanceToTarget < trackingDistanceThresholdMm)
                {
                    // If the fly is close enough to the center of the view,
                    // don't move. This helps avoid jittering, reduces wear on
                    // the motors, and reduces mechanical resonance.
                    continue;
                }

                double dx = physicalPosX - currentPhysicalPosX;
                double dy = physicalPosY - currentPhysicalPosY;
                MotionStagePosition targetMotionStagePosition = {
                    myMotionStagePosition.xPosMm + dx,
                    myMotionStagePosition.yPosMm + dy,
                    ABSOLUTE};

                // spdlog::debug(
                //     "Fly found at physical ({:.2}, {:.2}). "
                //     "Current center of view is at physical ({:.2}, {:.2}). "
                //     "dx={:.2}, dy={:.2}. ",
                //     physicalPosX,
                //     physicalPosY,
                //     currentPhysicalPosX,
                //     currentPhysicalPosY,
                //     dx,
                //     dy);
                setTargetMotionStagePosition(targetMotionStagePosition,
                                             defaultVelocity);
            }
        }
        else
        {
            // spdlog::debug("Tracking controller is overriding tracking.");
            MotionStagePosition currentPos = getCurrentMotionStagePosition();
            double distanceToTarget =
                calculateDistance(
                    trackingControlState->overridingPosX.load(),
                    trackingControlState->overridingPosY.load(),
                    currentPos.xPosMm,
                    currentPos.yPosMm);

            if (distanceToTarget < trackingDistanceThresholdMm &&
                checkIfMotionStageIdle())
            {
                trackingControlState->shouldOverrideTracking.store(false);
                continue;
            }
            else
            {
                MotionStagePosition targetPos = {
                    trackingControlState->overridingPosX.load(),
                    trackingControlState->overridingPosY.load(),
                    ABSOLUTE};
                setTargetMotionStagePosition(targetPos,
                                             defaultVelocity);
            }
        }
        uint64_t currentTime = getCurrentTimeMicroseconds();
        uint64_t elapsedTime = currentTime - startTime;
        long int timeToSleepMicrosecs = updateIntervalMicrosecs - elapsedTime;
        if (timeToSleepMicrosecs > 0)
        {
            std::this_thread::sleep_for(
                std::chrono::microseconds(timeToSleepMicrosecs));
        }
        else
        {
            spdlog::warn(
                "Tracking controller thread is running behind. "
                "I'm updating stage position at {} Hz, so I have only {} us) "
                "to complete each update. It took {} us this cycle. If this "
                "only happens sporadically, it's harmless.",
                trackingUpdateFrequency, updateIntervalMicrosecs, elapsedTime);
        }
    }
}

cv::Mat blackoutOutside(cv::Mat image,
                        MotionStagePosition stagePos,
                        CalibrationParams &behaviorCamCalibrationParams,
                        const RecorderConfig &recorderConfig)
{
    if (!behaviorCamCalibrationParams.isDefined)
    {
        // If calibration is not defined, don't do anything because we don't
        // know where the boundaries are in pixel coordinates
        return image;
    }

    double boundaryMarginMm = recorderConfig.getParameter<double>(
        "tracking", "boundary_margin_mm");
    double arenaSizeXMm = recorderConfig.getParameter<double>(
        "arena", "size_x_mm");
    double arenaSizeYMm = recorderConfig.getParameter<double>(
        "arena", "size_y_mm");
    double xMinPhysical = boundaryMarginMm;
    double xMaxPhysical = arenaSizeXMm - boundaryMarginMm;
    double yMinPhysical = boundaryMarginMm;
    double yMaxPhysical = arenaSizeYMm - boundaryMarginMm;

    int x0, y0, x1, y1, x2, y2, x3, y3;
    std::tie(y0, x0) =
        behaviorCamCalibrationParams.stagePosAndPhysicalPosToPixelPos(
            stagePos.xPosMm, stagePos.yPosMm, xMinPhysical, yMinPhysical);
    std::tie(y1, x1) =
        behaviorCamCalibrationParams.stagePosAndPhysicalPosToPixelPos(
            stagePos.xPosMm, stagePos.yPosMm, xMaxPhysical, yMinPhysical);
    std::tie(y2, x2) =
        behaviorCamCalibrationParams.stagePosAndPhysicalPosToPixelPos(
            stagePos.xPosMm, stagePos.yPosMm, xMaxPhysical, yMaxPhysical);
    std::tie(y3, x3) =
        behaviorCamCalibrationParams.stagePosAndPhysicalPosToPixelPos(
            stagePos.xPosMm, stagePos.yPosMm, xMinPhysical, yMaxPhysical);

    // Clamp values to image boundaries
    int numRows = image.rows;
    int numCols = image.cols;
    std::vector<cv::Point> corners = {
        cv::Point(x0, y0),
        cv::Point(x1, y1),
        cv::Point(x2, y2),
        cv::Point(x3, y3)};
    // Wrap it in a vector of vector for fillPoly
    std::vector<std::vector<cv::Point>> pts = {corners};

    // Create black image
    cv::Mat blackedOutImage = cv::Mat::zeros(image.size(), image.type());

    // Create mask for what's within the stage boundaries, with a boundary width
    // considered
    cv::fillPoly(blackedOutImage, pts, cv::Scalar(1));

    // Copy the ROI from the original image to the blacked out image
    image.copyTo(blackedOutImage, blackedOutImage);

    return blackedOutImage;
}

void motionStagePositionLogger(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<TrackingControlState> trackingControlState,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ProgramState> programState)
{
    int positionLoggingFreq = recorderConfig.getParameter<int>(
        "motion_control", "position_logging_frequency_hz");
    int loggingIntervalMicrosecs = 1e6 / positionLoggingFreq;

    std::set<std::string> initializedSaveDirectories; // root save directories
    std::ofstream logFile;

    while (!programState->toQuit.load())
    {
        // Get current position
        uint64_t startTime = getCurrentTimeMicroseconds();
        MotionStagePosition currentPosition = getCurrentMotionStagePosition();
        uint64_t endTime = getCurrentTimeMicroseconds();

        // Update latest position for other threads
        {
            std::lock_guard<std::mutex> lock(
                trackingControlState->latestMotionStagePositionMutex);
            trackingControlState->latestMotionStagePosition = currentPosition;
        }

        // Log position
        if (programState->isRecording.load())
        {
            if (initializedSaveDirectories.find(saveDirectory->getDirectory()) ==
                initializedSaveDirectories.end())
            {
                spdlog::info(
                    "Stage position log file not initialized under {}. "
                    "Creating one now.",
                    saveDirectory->getDirectory().c_str());
                logFile = initializeMotionStageLogFile(
                    saveDirectory->getDirectory());
                initializedSaveDirectories.insert(saveDirectory->getDirectory());
                spdlog::info("Stage position logging starts now!");
            }

            logFile << startTime << ","
                    << currentPosition.xPosMm << ","
                    << currentPosition.yPosMm << "\n";
            logFile.flush();
        }
        else
        {
            if (logFile.is_open())
            {
                spdlog::info("Stage position logging stopped. Closing log file.");
                logFile.close();
            }
        }

        // Wait for the next logging interval
        uint64_t elapsedTime = endTime - startTime;
        long int timeToSleepMicrosecs = loggingIntervalMicrosecs - elapsedTime;
        if (timeToSleepMicrosecs > 0)
        {
            std::this_thread::sleep_for(
                std::chrono::microseconds(timeToSleepMicrosecs));
        }
        else
        {
            spdlog::warn(
                "Motion stage position logging thread is running behind. "
                "I'm updating stage position at {} Hz, so I have only {} us) "
                "to complete each update. It took {} us this cycle. If this "
                "only happens sporadically, it's harmless.",
                positionLoggingFreq, loggingIntervalMicrosecs, elapsedTime);
        }
    }
    spdlog::info("Motion stage position logging thread stopped.");
}

std::tuple<bool, double, double> calculateFlyPositionAbsoluteMm(
    cv::Mat behaviorImage,
    MotionStagePosition stagePosition,
    CalibrationParams &behaviorCamCalibrationParams,
    const RecorderConfig &recorderConfig)
{
    bool isFound = false;
    double physicalPosXMm = 0;
    double physicalPosYMm = 0;

    if (behaviorImage.empty())
    {
        spdlog::warn(
            "Input behavior image is empty. It's normal if this happens "
            "only one or two times at the start of recording.");
        return {isFound, physicalPosXMm, physicalPosYMm};
    }
    if (!behaviorCamCalibrationParams.isDefined)
    {
        // Cannot map pixel positions to physical positions because the
        // calibration model has not been defined yet
        return {isFound, physicalPosXMm, physicalPosYMm};
    }

    // Remove pixels outside the stage boundaries
    cv::Mat blackedOutImage = blackoutOutside(behaviorImage,
                                              stagePosition,
                                              behaviorCamCalibrationParams,
                                              recorderConfig);

    assert(blackedOutImage.channels() == 1);

    // Threshold the image at a cutout of 100
    // spdlog::debug("Thresholding");
    cv::Mat binaryImage;
    cv::threshold(blackedOutImage,
                  binaryImage,
                  imageBinarizeThreshold,
                  255,
                  cv::THRESH_BINARY);
    if (binaryImage.empty())
    {
        spdlog::error("Binary image after thresholding is empty");
        return {isFound, physicalPosXMm, physicalPosYMm};
    }
    // display binaryImage for debugging
    // cv::imshow("binaryImage", binaryImage);
    if (cv::countNonZero(binaryImage) == 0)
    {
        return {isFound, physicalPosXMm, physicalPosYMm};
    }

    // Apply morphological opening and closing with a smaller kernel
    // spdlog::debug("Getting kernel for morphological operations");
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // spdlog::debug("Applying morphological opening");
    cv::Mat openedImage;
    cv::morphologyEx(binaryImage, openedImage, cv::MORPH_OPEN, kernel);

    // spdlog::debug("Applying morphological closing");
    cv::Mat morphedImage;
    cv::morphologyEx(openedImage, morphedImage, cv::MORPH_CLOSE, kernel);

    // spdlog::debug("Finding connected components");
    cv::Mat labels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(
        morphedImage, labels, stats, centroids);

    // Find the largest connected component (excluding the background which is
    // label 0).
    // spdlog::debug("Finding the largest connected component");
    int maxArea = 0;
    int maxLabel = 0;
    for (int i = 1; i < numLabels; i++)
    {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > maxArea)
        {
            maxArea = area;
            maxLabel = i;
        }
    }

    // If the largest connected component is large enough, this is the fly

    int minFlySizeSqPixels = recorderConfig.getParameter<int>(
        "tracking", "min_fly_size_sq_pixels");
    if (maxLabel > 0 && maxArea > minFlySizeSqPixels)
    {
        // spdlog::debug(
        //     "Getting the center of mass of the largest connected component");
        double centerOfMassCol = centroids.at<double>(maxLabel, 0); // x
        double centerOfMassRow = centroids.at<double>(maxLabel, 1); // y

        auto [x, y] = behaviorCamCalibrationParams
                          .stagePosAndPixelPosToPhysicalPos(
                              stagePosition.xPosMm,
                              stagePosition.yPosMm,
                              centerOfMassRow,
                              centerOfMassCol);
        isFound = true;
        physicalPosXMm = x;
        physicalPosYMm = y;

        // spdlog::debug(
        //     "Fly found at pixel ({:.2f}, {:.2f}), physical ({:.2f}, {:.2f})",
        //     centerOfMassCol, centerOfMassRow, physicalPosXMm, physicalPosYMm
        // );
    }

    return std::make_tuple(isFound, physicalPosXMm, physicalPosYMm);
}

MotionStagePosition getCurrentMotionStagePosition()
{
    size_t myThreadIdHash = getMyThreadIdHash();

    // Push request
    MotionStageRequest myRequest;
    myRequest.clientIdHash = myThreadIdHash;
    myRequest.requestType = GET_CURRENT_POSITION;
    {
        std::lock_guard<std::mutex> lock(requestMutex);
        requestQueue.push(myRequest);
    }
    requestCondVar.notify_one();

    // Wait for response
    MotionStageResponse myResponse;
    {
        std::unique_lock<std::mutex> lock(responseMutex);
        responseCondVar.wait(lock, [myThreadIdHash]
                             { return responseMap.find(myThreadIdHash) !=
                                      responseMap.end(); });
        myResponse = responseMap[myThreadIdHash];
        responseMap.erase(myThreadIdHash);
    }
    return myResponse.position;
}

void setTargetMotionStagePosition(MotionStagePosition targetPosition,
                                  float velocity)
{
    size_t myThreadIdHash = getMyThreadIdHash();

    // Push request
    MotionStageRequest myRequest;
    myRequest.clientIdHash = myThreadIdHash;
    myRequest.requestType = SET_TARGET_POSITION;
    myRequest.position = targetPosition;
    myRequest.velocity = velocity;
    {
        std::lock_guard<std::mutex> lock(requestMutex);
        requestQueue.push(myRequest);
    }
    requestCondVar.notify_one();

    // Wait for response
    MotionStageResponse myResponse;
    {
        std::unique_lock<std::mutex> lock(responseMutex);
        responseCondVar.wait(lock, [myThreadIdHash]
                             { return responseMap.find(myThreadIdHash) !=
                                      responseMap.end(); });
        myResponse = responseMap[myThreadIdHash];
        responseMap.erase(myThreadIdHash);
    }
    if (!myResponse.setSuccess)
    {
        spdlog::critical("Failed to set target motion stage position.");
        throw std::runtime_error(
            "Failed to set target motion stage position.");
    }
}

void waitUntilMotionStageIdleSync()
{
    size_t myThreadIdHash = getMyThreadIdHash();

    // Push request
    MotionStageRequest myRequest;
    myRequest.clientIdHash = myThreadIdHash;
    myRequest.requestType = WAIT_UNTIL_IDLE;
    {
        std::lock_guard<std::mutex> lock(requestMutex);
        requestQueue.push(myRequest);
    }
    requestCondVar.notify_one();

    // Wait for response
    MotionStageResponse myResponse;
    {
        std::unique_lock<std::mutex> lock(responseMutex);
        responseCondVar.wait(lock, [myThreadIdHash]
                             { return responseMap.find(myThreadIdHash) !=
                                      responseMap.end(); });
        myResponse = responseMap[myThreadIdHash];
        responseMap.erase(myThreadIdHash);
    }
    if (!myResponse.isIdle)
    {
        spdlog::critical(
            "Motion stage request handler thread responed to WAIT_UNTIL_IDLE "
            "request, but the stages are not idle.");
        throw std::runtime_error(
            "Motion stage request handler thread responed to WAIT_UNTIL_IDLE "
            "request, but the stages are not idle.");
    }
}

/**
 * @brief Waits asynchronously until the motion stage becomes idle.
 *
 * This function checks if the motion stage is idle by calling
 * `checkIfMotionStageIdle` every 500 ms. Like waitUntilMotionStageIdleSync(),
 * this function is blocking, but it doesn't block the thread that handles
 * requests to read form / write to the hardware, so other threads who need to
 * interface with the motion stages can still do it.
 */
void waitUntilMotionStageIdleAsync()
{
    while (!checkIfMotionStageIdle())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

bool checkIfMotionStageIdle()
{
    size_t myThreadIdHash = getMyThreadIdHash();

    // Push request
    MotionStageRequest myRequest;
    myRequest.clientIdHash = myThreadIdHash;
    myRequest.requestType = CHECK_IF_IDLE;
    {
        std::lock_guard<std::mutex> lock(requestMutex);
        requestQueue.push(myRequest);
    }
    requestCondVar.notify_one();

    // Wait for response
    MotionStageResponse myResponse;
    {
        std::unique_lock<std::mutex> lock(responseMutex);
        responseCondVar.wait(lock, [myThreadIdHash]
                             { return responseMap.find(myThreadIdHash) !=
                                      responseMap.end(); });
        myResponse = responseMap[myThreadIdHash];
        responseMap.erase(myThreadIdHash);
    }
    return myResponse.isIdle;
}

void startHomingMotionStage()
{
    size_t myThreadIdHash = getMyThreadIdHash();

    // Push request
    MotionStageRequest myRequest;
    myRequest.clientIdHash = myThreadIdHash;
    myRequest.requestType = START_HOMING;
    {
        std::lock_guard<std::mutex> lock(requestMutex);
        requestQueue.push(myRequest);
    }
    requestCondVar.notify_one();

    // Wait for response
    MotionStageResponse myResponse;
    {
        std::unique_lock<std::mutex> lock(responseMutex);
        responseCondVar.wait(lock, [myThreadIdHash]
                             { return responseMap.find(myThreadIdHash) !=
                                      responseMap.end(); });
        myResponse = responseMap[myThreadIdHash];
        responseMap.erase(myThreadIdHash);
    }
    if (!myResponse.setSuccess)
    {
        spdlog::critical("Failed to start homing motion stage.");
        throw std::runtime_error("Failed to start homing motion stage.");
    }
}

void stopMotionControlRequestHandler(std::shared_ptr<ProgramState> programState)
{
    if (!programState->toQuit.load())
    {
        spdlog::critical(
            "stopMotionControlRequestHandler() called but toQuit "
            "is not set to true. This shouldn't happen.");
        throw std::runtime_error(
            "stopMotionControlRequestHandler() called but toQuit "
            "is not set to true. This shouldn't happen.");
    }
    else
    {
        requestCondVar.notify_one();
    }
}