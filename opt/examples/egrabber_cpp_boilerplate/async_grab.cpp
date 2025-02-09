#include <EGrabber.h>
#include <FormatConverter.h>
#include <opencv2/opencv.hpp>

#define MAX_FRAMES 300

using namespace std;
using namespace Euresys;

int main(int argc, char **argv) {
    cout << "Running GenTL eGrabber discovery" << endl;
    EGenTL gen_tl;
    EGrabberDiscovery egrabber_discovery(gen_tl);
    egrabber_discovery.discover();

    cout << "Configuring camera" << endl;
    auto camera = egrabber_discovery.cameras(0);
    EGrabber<CallbackOnDemand> *frameGrabberPtr =
        new EGrabber<CallbackOnDemand>(camera);
    
    const int imageWidth = frameGrabberPtr->getInteger<RemoteModule>("Width");
    const int imageHeight = frameGrabberPtr->getInteger<RemoteModule>("Height");
    
    FormatConverter converter(gen_tl);

    for (int frameId = 0; frameId < MAX_FRAMES; ++frameId) {
        // Get data from camera/frame grabber
        ScopedBuffer buffer(*frameGrabberPtr);
        uint8_t *imagePtr = buffer.getInfo<uint8_t*>(gc::BUFFER_INFO_BASE);
        
        // Get the timestamp of the frame, in nanoseconds
        uint64_t timestamp = buffer.getInfo<uint64_t>(
            GenTL::BUFFER_INFO_TIMESTAMP
        );
        
        // Convert to OpenCV Mat and process in OpenCV
        cv::Mat frame(imageHeight, imageWidth, CV_8UC1, imagePtr);
        // ... do something with the frame

        // Display the frame live
        cv::imshow("JAI camera", frame);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    cv::destroyAllWindows();
}