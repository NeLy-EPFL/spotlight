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

    std::ofstream initializeMotionStageLogFile()
    {
        std::filesystem::path motionStageLogDir;
        {
            std::lock_guard<std::mutex> lock(isIOInitializing);
            motionStageLogDir = prepareOutputFolder(
                fs::path(saveDirectory) / "stage_position", true);
            spdlog::info("Motion stage log directory: {}",
                         motionStageLogDir.string());
        }
        std::filesystem::path filename =
            motionStageLogDir / "stage_position.csv";
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
}

void motionControlRequestHandler()
{
    MotionControl motionControl;
    motionControlHandlerReady.store(true);

    while (!toQuit.load())
    {
        MotionStageRequest myRequest;
        // Wait for a request
        {
            std::unique_lock<std::mutex> lock(requestMutex);
            requestCondVar.wait(lock, []
                                { return !requestQueue.empty() ||
                                         toQuit.load(); });

            // If there's still work to do, finish it even if told to stop
            if (!requestQueue.empty())
            {
                myRequest = requestQueue.front();
                requestQueue.pop();
            }
            else
            {
                // Only way to reach here is if toQuit is true
                assert(toQuit.load());
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

void trackingController()
{
    while (!motionControlHandlerReady.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    int updateFreq = 25;            // Hz
    double distanceThreshold = 0.1; // mm
    uint64_t updateIntervalMicrosecs = 1e6 / updateFreq;

    while (!toQuit.load())
    {
        uint64_t startTime = getCurrentTimeMicroseconds();

        if (!shouldOverrideTracking.load())
        {
            cv::Mat myBehaviorImage;
            {
                std::lock_guard<std::mutex> lock(latestFrameMutex);
                myBehaviorImage = latestFrameData.image;
            }
            MotionStagePosition myMotionStagePosition;
            {
                std::lock_guard<std::mutex> lock(latestMotionStagePositionMutex);
                myMotionStagePosition = latestMotionStagePosition;
            }

            auto [isFound, physicalPosX, physicalPosY] = calculateFlyPositionAbsoluteMm(
                myBehaviorImage, myMotionStagePosition);
            if (isFound)
            {
                auto [currentPhysicalPosX, currentPhysicalPosY] =
                    stagePosAndPixelPosToPhysicalPos(
                        myMotionStagePosition.xPosMm,
                        myMotionStagePosition.yPosMm,
                        myBehaviorImage.rows / 2,
                        myBehaviorImage.cols / 2);
                double dx = physicalPosX - currentPhysicalPosX;
                double dy = physicalPosY - currentPhysicalPosY;
                MotionStagePosition targetMotionStagePosition = {
                    myMotionStagePosition.xPosMm - dx, // note the flip
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
                setTargetMotionStagePosition(targetMotionStagePosition);
            }
        }
        else if (!isCalibrating.load())
        {
            spdlog::debug("Tracking controller is overriding tracking.");
            MotionStagePosition currentPos = getCurrentMotionStagePosition();
            double distanceToTarget =
                std::sqrt(
                    std::pow(overrideXPosAbsolute.load() - currentPos.xPosMm, 2) +
                    std::pow(overrideYPosAbsolute.load() - currentPos.yPosMm, 2));

            if ((distanceToTarget < distanceThreshold) && checkIfMotionStageIdle())
            {
                shouldOverrideTracking.store(false);
                continue;
            }
            else
            {
                MotionStagePosition targetPos = {
                    overrideXPosAbsolute.load(),
                    overrideYPosAbsolute.load(),
                    ABSOLUTE};
                setTargetMotionStagePosition(targetPos);
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
                updateFreq,
                updateIntervalMicrosecs,
                elapsedTime);
        }
    }
}

// cv::Mat blackoutOutside(cv::Mat image, MotionStagePosition stagePos)
// {
//     double xMinPhysical = 1;
//     double xMaxPhysical = 48 - 1;
//     double yMinPhysical = 1;
//     double yMaxPhysical = 72 - 1;

//     int rowMinPixel, rowMaxPixel, colMinPixel, colMaxPixel;
//     std::tie(rowMinPixel, colMinPixel) = stagePosAndPhysicalPosToPixelPos(
//         stagePos.xPosMm, stagePos.yPosMm, xMinPhysical, yMinPhysical);
//     std::tie(rowMaxPixel, colMaxPixel) = stagePosAndPhysicalPosToPixelPos(
//         stagePos.xPosMm, stagePos.yPosMm, xMaxPhysical, yMaxPhysical);

//     // TODO: Make a mask. Initialize it as all 255.
//     // Mark all rows < rowMinPixel as 0
//     // Mark all rows > rowMaxPixel as 0
//     // Mark all cols < colMinPixel as 0
//     // Mark all cols > colMaxPixel as 0

//     // TODO: Apply mask
//     // Keep only pixels in range

//     return blackedOutImage;
// }

cv::Mat blackoutOutside(cv::Mat image, MotionStagePosition stagePos)
{
    double xMinPhysical = 1;
    double xMaxPhysical = 48 - 1;
    double yMinPhysical = 1;
    double yMaxPhysical = 72 - 1;

    int xMinPixel, xMaxPixel, yMinPixel, yMaxPixel;
    std::tie(xMinPixel, yMinPixel) = stagePosAndPhysicalPosToPixelPos(
        stagePos.xPosMm, stagePos.yPosMm, xMinPhysical, yMinPhysical);
    std::tie(xMaxPixel, yMaxPixel) = stagePosAndPhysicalPosToPixelPos(
        stagePos.xPosMm, stagePos.yPosMm, xMaxPhysical, yMaxPhysical);
    std::swap(xMinPixel, xMaxPixel); // note: mirrored horizontally

    // Clamp values to image boundaries
    xMinPixel = std::max(0, xMinPixel);
    xMaxPixel = std::min(image.cols - 1, xMaxPixel); // Note the -1
    yMinPixel = std::max(0, yMinPixel);
    yMaxPixel = std::min(image.rows - 1, yMaxPixel); // Note the -1

    // Create black image
    cv::Mat blackedOutImage = cv::Mat::zeros(image.size(), image.type());

    // Width and height are +1 because the pixels are inclusive
    cv::Rect roi(xMinPixel,
                 yMinPixel,
                 xMaxPixel - xMinPixel + 1,
                 yMaxPixel - yMinPixel + 1);

    // Copy the ROI from the original image to the blacked out image
    // image(roi).copyTo(blackedOutImage(roi));
    // cv::Mat temp = image(roi).clone();
    // temp.copyTo(blackedOutImage(roi));
    for (int y = yMinPixel; y <= yMaxPixel; y++)
    {
        for (int x = xMinPixel; x <= xMaxPixel; x++)
        {
            if (image.channels() == 1)
            {
                blackedOutImage.at<uchar>(y, x) = image.at<uchar>(y, x);
            }
            else if (image.channels() == 3)
            {
                blackedOutImage.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(y, x);
            }
        }
    }

    return blackedOutImage;
}

void motionStagePositionLogger()
{
    int loggingIntervalMicrosecs = 1e6 / MOTION_STAGE_LOGGING_FREQUENCY_HZ;
    std::set<std::string> initializedSaveDirectories; // root save directories
    std::ofstream logFile;

    while (!toQuit.load())
    {
        // Get current position
        uint64_t startTime = getCurrentTimeMicroseconds();
        MotionStagePosition currentPosition = getCurrentMotionStagePosition();
        uint64_t endTime = getCurrentTimeMicroseconds();

        // Update latest position for other threads
        {
            std::lock_guard<std::mutex> lock(latestMotionStagePositionMutex);
            latestMotionStagePosition = currentPosition;
        }

        // Log position
        if (isRecording.load())
        {
            if (initializedSaveDirectories.find(saveDirectory) ==
                initializedSaveDirectories.end())
            {
                spdlog::info(
                    "Stage position log directory not initialized under {}. "
                    "Creating a folder now.",
                    saveDirectory);
                logFile = initializeMotionStageLogFile();
                initializedSaveDirectories.insert(saveDirectory);
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
                MOTION_STAGE_LOGGING_FREQUENCY_HZ,
                loggingIntervalMicrosecs,
                elapsedTime);
        }
    }
    spdlog::info("Motion stage position logging thread stopped.");
}

// MotionStagePosition calculateFlyPositionAbsoluteMm(
//     cv::Mat behaviorImage, MotionStagePosition stagePosition)
// {
//     cv::Mat blackedOutImage = blackoutOutside(behaviorImage, stagePosition);

//     // Threshold the image at a cutout of 100; apply morphological opening and
//     // closing to remove noise (let's say with a 9x9 kernel); find the largest
//     // connected component; find the center of mass of the connected component
//     // in pixel x-y (ie. col-row) coordinates. Then, convert the pixel x-y
//     // coordinates and stage x-y coordinates to physical coordinates using
//     // the following
//     auto [physicalPosXMm, physicalPosYMm] = stagePosAndPixelPosToPhysicalPos(
//         stagePosition.xPosMm, stagePosition.yPosMm, centerOfMassRow, centerOfMassCol);
// }

std::tuple<bool, double, double> calculateFlyPositionAbsoluteMm(
    cv::Mat behaviorImage, MotionStagePosition stagePosition)
{
    bool isFound = false;
    double physicalPosXMm = 0;
    double physicalPosYMm = 0;

    if (behaviorImage.empty())
    {
        if (behaviorCameraReady.load())
        {
            spdlog::error("Input behavior image is empty");
        }
        return {isFound, physicalPosXMm, physicalPosYMm};
    }

    cv::Mat correctedImage = correctImageRotationAndFlip(behaviorImage);

    // Remove pixels outside the stage boundaries
    cv::Mat blackedOutImage = blackoutOutside(correctedImage.clone(),
                                              stagePosition);

    assert(blackedOutImage.channels() == 1);

    // Threshold the image at a cutout of 100
    // spdlog::debug("Thresholding");
    cv::Mat binaryImage;
    cv::threshold(blackedOutImage, binaryImage, 100, 255, cv::THRESH_BINARY);
    if (binaryImage.empty())
    {
        // spdlog::error("Binary image after thresholding is empty");
        // toQuit.store(true);
        return {isFound, physicalPosXMm, physicalPosYMm};
    }
    // display binaryImage for debugging
    // cv::imshow("binaryImage", binaryImage);
    if (cv::countNonZero(binaryImage) == 0)
    {
        // spdlog::debug("Binary image after thresholding has no non-zero pixels");
        // toQuit.store(true);
        return {isFound, physicalPosXMm, physicalPosYMm};
    }

    // Apply morphological opening and closing with a smaller kernel
    // spdlog::debug("Getting kernel for morphological operations");
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); // Smaller kernel

    // spdlog::debug("Applying morphological opening");
    cv::Mat openedImage;
    cv::morphologyEx(binaryImage, openedImage, cv::MORPH_OPEN, kernel);

    // spdlog::debug("Applying morphological closing");
    cv::Mat morphedImage;
    cv::morphologyEx(openedImage, morphedImage, cv::MORPH_CLOSE, kernel);

    // spdlog::debug("Finding connected components");
    cv::Mat labels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(morphedImage, labels, stats, centroids);

    // Find the largest connected component (excluding the background which is label 0)
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

    // If the largest connected component is at least 200 pixels, this is the fly
    if (maxLabel > 0 && maxArea > 200)
    {
        // spdlog::debug("Getting the center of mass of the largest connected component");
        double centerOfMassCol = centroids.at<double>(maxLabel, 0); // x coordinate
        double centerOfMassRow = centroids.at<double>(maxLabel, 1); // y coordinate

        auto [x, y] = stagePosAndPixelPosToPhysicalPos(
            stagePosition.xPosMm, stagePosition.yPosMm, centerOfMassCol, centerOfMassRow);
        isFound = true;
        physicalPosXMm = x;
        physicalPosYMm = y;

        // spdlog::debug("Fly found at pixel ({:.2f}, {:.2f}), physical ({:.2f}, {:.2f})",
        //               centerOfMassCol, centerOfMassRow, physicalPosXMm, physicalPosYMm);
    }

    // isFound = false; // TODO: remove this line
    return std::make_tuple(isFound, physicalPosXMm, physicalPosYMm);
}

// MotionStagePosition calculateFlyPositionAbsoluteMm(
//     cv::Mat behaviorImage, MotionStagePosition stagePosition)
// {
//     MotionStagePosition currentPosition = getCurrentMotionStagePosition();

//     // spdlog::debug("Calculating fly position in absolute mm");

//     // Check if input image is valid
//     if (behaviorImage.empty()) {
//         spdlog::error("Input behavior image is empty");
//         return stagePosition;
//     }

//     // Make sure we're working with a deep copy of the input image
//     cv::Mat blackedOutImage = blackoutOutside(behaviorImage.clone(), stagePosition);

//     // Check if blackedOutImage is valid
//     if (blackedOutImage.empty()) {
//         spdlog::error("Blacked out image is empty");
//         return stagePosition;
//     }

//     // Convert to grayscale if it's a color image
//     // spdlog::debug("Converting image to grayscale");
//     cv::Mat grayImage;
//     if (blackedOutImage.channels() > 1) {
//         cv::cvtColor(blackedOutImage, grayImage, cv::COLOR_BGR2GRAY);
//     } else {
//         grayImage = blackedOutImage.clone();
//     }

//     // Check if grayImage is valid
//     if (grayImage.empty()) {
//         spdlog::error("Gray image is empty");
//         return stagePosition;
//     }

//     // 1. Threshold the image at a cutout of 100
//     // spdlog::debug("Thresholding");
//     cv::Mat binaryImage;
//     cv::threshold(grayImage, binaryImage, 100, 255, cv::THRESH_BINARY);

//     // Check if binaryImage is valid and has non-zero pixels
//     if (binaryImage.empty()) {
//         spdlog::debug("Binary image after thresholding is empty");
//         return stagePosition;
//     }

//     // 2. Apply morphological opening and closing with a smaller kernel
//     // spdlog::debug("Getting kernel for morphological operations");
//     cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); // Smaller kernel

//     // Use separate matrices for input and output of morphological operations
//     // spdlog::debug("Applying morphological opening");
//     cv::Mat openedImage;

//     try {
//         cv::morphologyEx(binaryImage, openedImage, cv::MORPH_OPEN, kernel);

//         // spdlog::debug("Applying morphological closing");
//         cv::Mat morphedImage;
//         cv::morphologyEx(openedImage, morphedImage, cv::MORPH_CLOSE, kernel);

//         // 3. Find connected components
//         // spdlog::debug("Finding connected components");
//         cv::Mat labels, stats, centroids;
//         int numLabels = cv::connectedComponentsWithStats(morphedImage, labels, stats, centroids);

//         // 4. Find the largest connected component (excluding the background which is label 0)
//         // spdlog::debug("Finding the largest connected component");
//         int maxArea = 0;
//         int maxLabel = 0;
//         for (int i = 1; i < numLabels; i++) {
//             int area = stats.at<int>(i, cv::CC_STAT_AREA);
//             if (area > maxArea) {
//                 maxArea = area;
//                 maxLabel = i;
//             }
//         }

//         if (maxArea < 20*20) {
//             // spdlog::debug("Largest connected component is too small");
//             return currentPosition;
//         }

//         // 5. Get the center of mass of the largest component
//         if (maxLabel > 0 && maxArea > 0) {  // Ensure we have a valid component
//             // spdlog::debug("Getting the center of mass of the largest connected component");
//             double centerOfMassCol = centroids.at<double>(maxLabel, 0); // x coordinate
//             double centerOfMassRow = centroids.at<double>(maxLabel, 1); // y coordinate

//             // 6. Convert pixel coordinates to physical coordinates
//             // spdlog::debug("Converting pixel coordinates to physical coordinates");
//             auto [physicalPosXMm, physicalPosYMm] = stagePosAndPixelPosToPhysicalPos(
//                 stagePosition.xPosMm, stagePosition.yPosMm, centerOfMassRow, centerOfMassCol);

//             // Update the position
//             MotionStagePosition flyPosition;
//             flyPosition.xPosMm = physicalPosXMm;
//             flyPosition.yPosMm = physicalPosYMm;

//             return flyPosition;
//         }

//         return stagePosition;
//     }
//     catch (const cv::Exception& e) {
//         spdlog::error("OpenCV exception: {}", e.what());
//         return stagePosition;
//     }
// }

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

void setTargetMotionStagePosition(
    MotionStagePosition targetPosition, float velocity)
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

void stopMotionControlRequestHandler()
{
    if (!toQuit.load())
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

void runCalibrationScanProcedureOneDirection(
    int currentlySetExposureTimeMicrosecs,
    CalibrationScanDirection scanDirection)
{
    bool isByRow = scanDirection == ROW_BY_ROW;
    isCalibrating.store(true);
    shouldOverrideTracking.store(true);

    // Start recording
    spdlog::info("Starting recording for calibration scan.");
    std::string directionStr = isByRow ? "row_by_row" : "column_by_column";
    fs::path scanSaveDirectory = prepareOutputFolder(
        fs::path(SPOTLIGHT_ARUCO_SCAN_DIR) / directionStr,
        true); // mkdir -p
    saveDirectory = scanSaveDirectory.string();
    spdlog::debug("Setting saveDirectory to {}", saveDirectory);
    // Wait for the reset directory to take effect
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    triggerController->startRecording(
        CALIBRATION_SCAN_FPS, CALIBRATION_SCAN_EXPOSURE_TIME_MICROSECS);
    spdlog::info("Recording started for calibration scan.");

    // Go to the corner of the stage
    spdlog::info("Moving to the corner of the stage.");
    MotionStagePosition cornerPosition = {MOTION_STAGE_X_MIN_PHYSICAL_MM,
                                          MOTION_STAGE_Y_MIN_PHYSICAL_MM,
                                          ABSOLUTE};
    setTargetMotionStagePosition(cornerPosition,
                                 CALIBRATION_SCAN_STAGE_SPEED);
    waitUntilMotionStageIdleAsync();
    spdlog::info("Moved to the corner of the stage.");

    // Scan row by row
    bool isSecondaryAxisAtMin = true;
    float primaryAxisMin, primaryAxisMax, secondaryAxisMin, secondaryAxisMax;
    if (isByRow)
    {
        primaryAxisMin = MOTION_STAGE_Y_MIN_PHYSICAL_MM;
        primaryAxisMax = MOTION_STAGE_Y_MAX_PHYSICAL_MM;
        secondaryAxisMin = MOTION_STAGE_X_MIN_PHYSICAL_MM;
        secondaryAxisMax = MOTION_STAGE_X_MAX_PHYSICAL_MM;
    }
    else
    {
        primaryAxisMin = MOTION_STAGE_X_MIN_PHYSICAL_MM;
        primaryAxisMax = MOTION_STAGE_X_MAX_PHYSICAL_MM;
        secondaryAxisMin = MOTION_STAGE_Y_MIN_PHYSICAL_MM;
        secondaryAxisMax = MOTION_STAGE_Y_MAX_PHYSICAL_MM;
    }
    spdlog::debug("primaryAxisMin: {}, primaryAxisMax: {}, "
                  "secondaryAxisMin: {}, secondaryAxisMax: {}",
                  primaryAxisMin, primaryAxisMax,
                  secondaryAxisMin, secondaryAxisMax);

    for (float primaryPos = primaryAxisMin;
         primaryPos < primaryAxisMax;
         primaryPos += CALIBRATION_SCAN_STRIDE_MM)
    {
        float secondaryPos;
        MotionStagePosition targetPosition;

        // Move to the next row
        spdlog::debug("primaryPos: {}, "
                      "secondaryAxisMin: {}, secondaryAxisMax: {}",
                      primaryPos, secondaryAxisMin, secondaryAxisMax);
        if (isSecondaryAxisAtMin)
        {
            secondaryPos = secondaryAxisMin;
        }
        else
        {
            secondaryPos = secondaryAxisMax;
        }
        if (isByRow)
        {
            targetPosition = {secondaryPos, primaryPos, ABSOLUTE};
        }
        else
        {
            targetPosition = {primaryPos, secondaryPos, ABSOLUTE};
        }
        spdlog::debug("secondaryPos: {}", secondaryPos);
        setTargetMotionStagePosition(targetPosition,
                                     CALIBRATION_SCAN_STAGE_SPEED);
        waitUntilMotionStageIdleAsync();

        // Scan the row
        if (isSecondaryAxisAtMin)
        {
            secondaryPos = secondaryAxisMax;
        }
        else
        {
            secondaryPos = secondaryAxisMin;
        }
        if (isByRow)
        {
            targetPosition = {secondaryPos, primaryPos, ABSOLUTE};
        }
        else
        {
            targetPosition = {primaryPos, secondaryPos, ABSOLUTE};
        }
        setTargetMotionStagePosition(targetPosition,
                                     CALIBRATION_SCAN_STAGE_SPEED);
        waitUntilMotionStageIdleAsync();

        isSecondaryAxisAtMin = !isSecondaryAxisAtMin;
    }

    // Stop recording (reset exposure time to the way it was)
    triggerController->stopRecording(currentlySetExposureTimeMicrosecs);

    shouldOverrideTracking.store(false);
    isCalibrating.store(false);
    spdlog::info("Recording stopped for calibration scan.");
}

void runCalibrationScanProcedure(int currentlySetExposureTimeMicrosecs)
{
    runCalibrationScanProcedureOneDirection(
        currentlySetExposureTimeMicrosecs, ROW_BY_ROW);
    runCalibrationScanProcedureOneDirection(
        currentlySetExposureTimeMicrosecs, COLUMN_BY_COLUMN);
}