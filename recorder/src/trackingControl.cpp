#include "trackingControl.hpp"

namespace
{
    std::mutex motionStageRequestMutex;
    std::condition_variable motionStageRequestCondVar;
    std::mutex motionStageResponseMutex;
    std::condition_variable motionStageResponseCondVar;
    std::atomic<bool> newRequestForMotionstage;
    std::atomic<bool> newPositionFromMotionStage;
    MotionStageRequest latestMotionStageRequest;
    MotionStagePosition latestMotionStagePosition;
}

void motionControlRequestHandler()
{
    auto [numMicrosecsAllowedGet, numMicrosecsAllowedSet] =
        calculateMaxMotionStageRequestHandlingTime();

    MotionControl motionControl;
    motionControlHandlerReady.store(true);

    while (!toQuit.load()) // TODO: remove this?
    {
        MotionStageRequest myRequest;
        {
            std::unique_lock<std::mutex> lock(motionStageRequestMutex);
            motionStageRequestCondVar.wait(
                lock, []
                { return newRequestForMotionstage.load() || toQuit.load(); });

            if (toQuit.load())
            {
                spdlog::info(
                    "motion control request handler thread "
                    "is breaking out of loop.");
                break;
            }

            myRequest = latestMotionStageRequest;
            newRequestForMotionstage = false;
        }

        uint64_t startTime = getCurrentTimeMicroseconds();

        if (myRequest.requestType == SET_TARGET_POSITION)
        {
            motionControl.moveAbsolute(
                X_AXIS, myRequest.position.xPosAbsoluteMm, false);
            motionControl.moveAbsolute(
                Y_AXIS, myRequest.position.yPosAbsoluteMm, false);
        }
        else if (myRequest.requestType == GET_CURRENT_POSITION)
        {
            MotionStagePosition currentPosition;
            currentPosition.xPosAbsoluteMm =
                motionControl.getPosition(X_AXIS);
            currentPosition.yPosAbsoluteMm =
                motionControl.getPosition(Y_AXIS);
            {
                std::lock_guard<std::mutex> lock(motionStageResponseMutex);
                latestMotionStagePosition = currentPosition;
                newPositionFromMotionStage = true;
            }
            motionStageResponseCondVar.notify_one();
        }
        else if (myRequest.requestType == HOME)
        {
            motionControl.home(X_AXIS);
            motionControl.home(Y_AXIS);
            motionControl.waitUntilIdle(X_AXIS);
            motionControl.waitUntilIdle(Y_AXIS);
        }
        else
        {
            spdlog::error("Unknown motion stage request type {}",
                          myRequest.requestType);
        }

        uint64_t walltime = getCurrentTimeMicroseconds() - startTime;

        int walltimeLimit;
        switch (myRequest.requestType)
        {
        case GET_CURRENT_POSITION:
            walltimeLimit = numMicrosecsAllowedGet;
            break;
        case SET_TARGET_POSITION:
            walltimeLimit = numMicrosecsAllowedSet;
            break;
        default:
            walltimeLimit = INT_MAX;
            break;
        }

        if (walltime > walltimeLimit)
        {
            spdlog::warn(
                "Motion control request handler thread taking more than "
                "allotted time to handle request type {}; walltime {} us "
                "(allowed {} us). Note that the allowed time is only the "
                "AVERAGE amount of time that this thread can spend on each "
                "operation. It's harmless if this message appears only "
                "sporadically.",
                myRequest.requestType, walltime, walltimeLimit);
        }
    }

    spdlog::info("Motion control request handler thread stopped");
}

MotionStagePosition getCurrentMotionStagePosition()
{
    MotionStageRequest request;
    request.requestType = GET_CURRENT_POSITION;
    {
        std::lock_guard<std::mutex> lock(motionStageRequestMutex);
        latestMotionStageRequest = request;
        newRequestForMotionstage = true;
    }
    motionStageRequestCondVar.notify_one();

    {
        std::unique_lock<std::mutex> lock(motionStageResponseMutex);
        motionStageResponseCondVar.wait(
            lock, []
            { return newPositionFromMotionStage.load() || toQuit.load(); });
        newPositionFromMotionStage = false;
    }

    return latestMotionStagePosition;
}

void setTargetMotionStagePosition(MotionStagePosition targetPosition)
{
    MotionStageRequest request;
    request.requestType = SET_TARGET_POSITION;
    request.position = targetPosition;
    {
        std::lock_guard<std::mutex> lock(motionStageRequestMutex);
        latestMotionStageRequest = request;
        newRequestForMotionstage = true;
    }
    motionStageRequestCondVar.notify_one();
}

void flyTrackingController(int tolerancePx, int gainPx)
{
    // TODO: Implement fly tracking controller
    spdlog::info("Fly tracking controller not yet implemented");
}

void runCalibrationScanProcedure()
{
    // TODO: Implement calibration scan procedure
    spdlog::info("Calibration scan procedure not yet implemented");
}

void stopMotionControlRequestHandler()
{
    if (!toQuit.load())
    {
        spdlog::error("stopMotionControlRequestHandler() called but toQuit is "
                      "not set to true. This shouldn't happen.");
    }
    else
    {
        motionStageRequestCondVar.notify_all();
    }
}