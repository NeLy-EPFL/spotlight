#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
    std::cout << cv::getBuildInformation() << std::endl;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return 1;
    }

    // Load the image
    cv::Mat image = cv::imread(argv[1]);
    
    if (image.empty()) {
        std::cerr << "Error: Could not open or find the image!" << std::endl;
        return 1;
    }

    // Display the image
    cv::imshow("OpenCV Image", image);

    // Wait for a key press
    cv::waitKey(0);

    return 0;
}

