#include <thread>
#include <chrono>
#include <EGrabber.h>
#include <FormatConverter.h>
#include <opencv2/opencv.hpp>

#define MAX_FRAMES 3600 * 50
#define IMAGE_WIDTH_DESIRED 640
#define IMAGE_HEIGHT_DESIRED 480

using namespace std;
using namespace Euresys;

uint64_t getCurrentTimeMicroseconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

int roundToMultiplesOf64(int value)
{
    int remainder = value % 64;
    return value - remainder + (remainder > 32 ? 64 : 0);
}

void imageAcquirer(
    EGrabber<CallbackOnDemand> *frameGrabberPtr,
    int imageWidth,
    int imageHeight)
{
    frameGrabberPtr->reallocBuffers(20);
    frameGrabberPtr->start(MAX_FRAMES);
    double lastCheckpointTime = getCurrentTimeMicroseconds();
    for (int frameId = 0; frameId < MAX_FRAMES; ++frameId)
    {
        if (frameId % 100 == 0)
        {
            double currentTime = getCurrentTimeMicroseconds();
            double elapsedTime = (currentTime - lastCheckpointTime) / 1000000;
            double fps = 100.0 / elapsedTime;
            cout << "  Frame " << frameId << " - FPS: " << fps << endl;
            lastCheckpointTime = currentTime;
        }
        uint64_t timeStart = getCurrentTimeMicroseconds();
        ScopedBuffer buffer(*frameGrabberPtr);  // the main blocking call
        uint64_t timeGotBuffer = getCurrentTimeMicroseconds();
        uint8_t *imagePtr = buffer.getInfo<uint8_t *>(gc::BUFFER_INFO_BASE);
        uint64_t timeGotImagePtr = getCurrentTimeMicroseconds();
        cv::Mat frame(imageHeight, imageWidth, CV_8UC1, imagePtr);
        uint64_t timeConvertedToMat = getCurrentTimeMicroseconds();
        
        // It takes about 8000us to display the image, so to get a good idea
        // how long each step takes, comment out the following lines
        cv::imshow("JAI camera", frame);
        if (cv::waitKey(1) == 27)
        {
            break;
        }
        // ^^^^^^^^^^

        // The following numbers are taken from a run with the cv::imshow
        // code above commented out. The triggering signal is at 50Hz.
        // Taken together, it takes ~60us to get a single frame (excluding
        // the time to push it to some kind of queue).
        cout << "  Time spent getting buffer: "
             << timeGotBuffer - timeStart << " us" << endl;  // ~19950us
        cout << "  Time spent getting image pointer: "
             << timeGotImagePtr - timeGotBuffer << " us" << endl;  // ~5us
        cout << "  Time spent converting to cv::Mat: "
             << timeConvertedToMat - timeGotImagePtr << " us" << endl;  // ~2us
    }

    cout << "Stopping acquisition..." << endl;
    cv::destroyAllWindows();
}

int main(int argc, char **argv)
{
    cout << "Running GenTL eGrabber discovery..." << endl;
    EGenTL genTL;
    EGrabberDiscovery egrabberDiscovery(genTL);
    egrabberDiscovery.discover();
    cout << "  OK - GenTL eGrabber discovery completed" << endl;

    cout << "Configuring camera..." << endl;
    EGrabberCameraInfo camera = egrabberDiscovery.cameras(0);
    EGrabber<CallbackOnDemand> frameGrabber(camera);
    const int fullFrameWidth = frameGrabber.getInteger<RemoteModule>("Width");
    const int fullFrameHeight = frameGrabber.getInteger<RemoteModule>("Height");
    cout << "  OK - Camera configured" << endl;
    cout << "  Full frame width: " << fullFrameWidth << endl;
    cout << "  Full frame height: " << fullFrameHeight << endl;

    cout << "Setting sensor ROI..." << endl;
    int imageWidth = roundToMultiplesOf64(IMAGE_WIDTH_DESIRED);
    int imageHeight = roundToMultiplesOf64(IMAGE_HEIGHT_DESIRED);
    int xOffset = roundToMultiplesOf64((fullFrameHeight - imageHeight) / 2);
    int yOffset = roundToMultiplesOf64((fullFrameWidth - imageWidth) / 2);
    cout << "  Rounding everything to multiples of 64 for compatibility reasons" << endl;
    cout << "  Setting image width to: " << imageWidth << endl;
    cout << "  Setting image height to: " << imageHeight << endl;
    cout << "  Setting x offset to: " << xOffset << endl;
    cout << "  Setting y offset to: " << yOffset << endl;
    frameGrabber.setInteger<RemoteModule>("Width", imageWidth);
    frameGrabber.setInteger<RemoteModule>("Height", imageHeight);
    frameGrabber.setInteger<RemoteModule>("OffsetX", xOffset);
    frameGrabber.setInteger<RemoteModule>("OffsetY", yOffset);
    int actualImageWidth = frameGrabber.getInteger<RemoteModule>("Width");
    int actualImageHeight = frameGrabber.getInteger<RemoteModule>("Height");
    int actualXOffset = frameGrabber.getInteger<RemoteModule>("OffsetX");
    int actualYOffset = frameGrabber.getInteger<RemoteModule>("OffsetY");
    assert(actualImageWidth == imageWidth);
    assert(actualImageHeight == imageHeight);
    assert(actualXOffset == xOffset);
    assert(actualYOffset == yOffset);
    cout << "  OK - Successfully set sensor ROI" << endl;

    cout << "Configuring trigger-related settings..." << endl;
    // For this, please feed a trigger signal (high = active, exposure time
    // controlled by trigger width) to the TTLIO12 line (external IO plug) of
    // the frame grabber.
    cout << "  Setting TTLIO12 line as Input..." << endl;
    frameGrabber.setString<InterfaceModule>("LineSelector", "TTLIO12");
    frameGrabber.setString<InterfaceModule>("LineMode", "Input");

    cout << "  Setting LIN1 to TTLIO12..." << endl;
    frameGrabber.setString<InterfaceModule>("LineInputToolSelector", "LIN1");
    frameGrabber.setString<InterfaceModule>("LineInputToolSource", "TTLIO12");

    cout << "  Setting CameraControlMethod to EXTERNAL..." << endl;
    frameGrabber.setString<DeviceModule>("CameraControlMethod", "EXTERNAL");

    cout << "  Disabling trigger for AcquisitionStart and AcquisitionEnd..." << endl;
    frameGrabber.setString<RemoteModule>("TriggerSelector", "AcquisitionStart");
    frameGrabber.setString<RemoteModule>("TriggerMode", "Off");
    frameGrabber.setString<RemoteModule>("TriggerSelector", "AcquisitionEnd");
    frameGrabber.setString<RemoteModule>("TriggerMode", "Off");

    cout << "  Enabling trigger for FrameStart using CXPin as source..." << endl;
    frameGrabber.setString<RemoteModule>("TriggerSelector", "FrameStart");
    // frameGrabber.setString<RemoteModule>("TriggerMode", "On");
    frameGrabber.setString<RemoteModule>("TriggerSource", "CXPin");

    cout << "  Setting ExposureMode to TriggerWidth..." << endl;
    frameGrabber.setString<RemoteModule>("ExposureMode", "TriggerWidth");

    cout << "  OK - Successfully configured all trigger-related settings" << endl;

    cout << "Using CXP6 for faster data transmission..." << endl;
    frameGrabber.setString<RemoteModule>("LinkConfig", "CXP6_X4");
    cout << "  OK - Successfully set LinkConfig to CXP6_X4" << endl;

    FormatConverter converter(genTL);

    // Get data in a separate thread
    thread acquireThread(imageAcquirer, &frameGrabber, imageWidth, imageHeight);
    acquireThread.join();

    cout << "  OK - All done" << endl;
}