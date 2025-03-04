#include <tuple>

std::tuple<double, double> stagePosAndPixelPosToPhysicalPos(
    double stagePosX,
    double stagePosY,
    int pixelPosRow,
    int pixelPosCol);

std::tuple<int, int> stagePosAndPhysicalPosToPixelPos(
    double stagePosX,
    double stagePosY,
    double physicalPosX,
    double physicalPosY);