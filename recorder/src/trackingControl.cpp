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

void waitUntilMotionStageIdle()
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
    MotionStagePosition cornerPosition = {MOTION_STAGE_X_MIN_PHYSICAL_MM,
                                          MOTION_STAGE_Y_MIN_PHYSICAL_MM,
                                          ABSOLUTE};
    setTargetMotionStagePosition(cornerPosition);
    waitUntilMotionStageIdle();

    // Start recording
    fs::path scanSaveDirectory = prepareOutputFolder(SPOTLIGHT_ARUCO_SCAN_DIR,
                                                     true); // mkdir -p
    saveDirectory = scanSaveDirectory.string();
    triggerController->startRecording(
        CALIBRATION_SCAN_FPS, CALIBRATION_SCAN_EXPOSURE_TIME_MICROSECS);

    // Scan column by column
    bool isXAtMin = true;
    for (float yPos = MOTION_STAGE_Y_MIN_PHYSICAL_MM;
         yPos < MOTION_STAGE_Y_MAX_PHYSICAL_MM;
         yPos += CALIBRATION_SCAN_STRIDE_MM)
    {
        // Move to the next column
        float xPos = isXAtMin ? MOTION_STAGE_X_MIN_PHYSICAL_MM
                              : MOTION_STAGE_X_MAX_PHYSICAL_MM;
        setTargetMotionStagePosition({xPos, yPos, ABSOLUTE});
        waitUntilMotionStageIdle();

        // Scan the column
        xPos = isXAtMin ? MOTION_STAGE_X_MAX_PHYSICAL_MM
                        : MOTION_STAGE_X_MIN_PHYSICAL_MM;
        setTargetMotionStagePosition({xPos, yPos, ABSOLUTE});
        waitUntilMotionStageIdle();
    }

    // Stop recording (reset exposure time to the way it was)
    triggerController->stopRecording(currentlySetExposureTimeMicrosecs);
}