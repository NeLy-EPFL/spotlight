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

void runCalibrationScanProcedure(int currentlySetExposureTimeMicrosecs)
{
    // Go to the corner of the stage
    spdlog::info("Moving to the corner of the stage.");
    MotionStagePosition cornerPosition = {MOTION_STAGE_X_MIN_PHYSICAL_MM,
                                          MOTION_STAGE_Y_MIN_PHYSICAL_MM,
                                          ABSOLUTE};
    setTargetMotionStagePosition(cornerPosition,
                                 CALIBRATION_SCAN_STAGE_SPEED);
    waitUntilMotionStageIdleAsync();
    spdlog::info("Moved to the corner of the stage.");

    // Start recording
    spdlog::info("Starting recording for calibration scan.");
    fs::path scanSaveDirectory = prepareOutputFolder(SPOTLIGHT_ARUCO_SCAN_DIR,
                                                     true); // mkdir -p
    saveDirectory = scanSaveDirectory.string();
    triggerController->startRecording(
        CALIBRATION_SCAN_FPS, CALIBRATION_SCAN_EXPOSURE_TIME_MICROSECS);
    spdlog::info("Recording started for calibration scan.");

    // Scan row by row
    bool isXAtMin = true;
    for (float yPos = MOTION_STAGE_Y_MIN_PHYSICAL_MM;
         yPos < MOTION_STAGE_Y_MAX_PHYSICAL_MM;
         yPos += CALIBRATION_SCAN_STRIDE_MM)
    {
        // Move to the next row
        float xPos = isXAtMin ? MOTION_STAGE_X_MIN_PHYSICAL_MM
                              : MOTION_STAGE_X_MAX_PHYSICAL_MM;
        setTargetMotionStagePosition({xPos, yPos, ABSOLUTE},
                                     CALIBRATION_SCAN_STAGE_SPEED);
        waitUntilMotionStageIdleAsync();

        // Scan the row
        xPos = isXAtMin ? MOTION_STAGE_X_MAX_PHYSICAL_MM
                        : MOTION_STAGE_X_MIN_PHYSICAL_MM;
        setTargetMotionStagePosition({xPos, yPos, ABSOLUTE},
                                     CALIBRATION_SCAN_STAGE_SPEED);
        waitUntilMotionStageIdleAsync();

        isXAtMin = !isXAtMin;
    }

    // Stop recording (reset exposure time to the way it was)
    triggerController->stopRecording(currentlySetExposureTimeMicrosecs);
}