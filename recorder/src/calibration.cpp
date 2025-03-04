#include "calibration.hpp"

std::tuple<double, double> stagePosAndPixelPosToPhysicalPos(
    double stagePosX, double stagePosY, int pixelPosRow, int pixelPosCol)
{
    // array([[-1.00612784e+00,  2.06745867e-03, -1.00007277e-02,
    //     -7.27075788e-05,  1.17869514e+02],
    //    [ 5.51134434e-03,  1.00529191e+00, -7.93647261e-05,
    //      1.00037441e-02, -5.22831388e+01]])
    double physicalPosX =
        -1.00612784e+00 * stagePosX +
        2.06745867e-03 * stagePosY +
        -1.00007277e-02 * pixelPosRow +
        -7.27075788e-05 * pixelPosCol +
        1.17869514e+02;

    double physicalPosY =
        5.51134434e-03 * stagePosX +
        1.00529191e+00 * stagePosY +
        -7.93647261e-05 * pixelPosRow +
        1.00037441e-02 * pixelPosCol +
        -5.22831388e+01;

    return std::make_tuple(physicalPosX, physicalPosY);
}

std::tuple<int, int> stagePosAndPhysicalPosToPixelPos(
    double stagePosX,
    double stagePosY,
    double physicalPosX,
    double physicalPosY)
{
    // array([[-1.00595655e+02,  9.37273442e-01, -9.99869562e+01,
    //     -7.26708864e-01,  1.17474193e+04],
    //    [-1.34900401e+00, -1.00484130e+02, -7.93246740e-01,
    //      9.99568077e+01,  5.31955526e+03]])
    int pixelPosRow =
        -1.00595655e+02 * stagePosX +
        9.37273442e-01 * stagePosY +
        -9.99869562e+01 * physicalPosX +
        -7.26708864e-01 * physicalPosY +
        1.17474193e+04;

    int pixelPosCol =
        -1.34900401e+00 * stagePosX +
        -1.00484130e+02 * stagePosY +
        -7.93246740e-01 * physicalPosX +
        9.99568077e+01 * physicalPosY +
        5.31955526e+03;

    return std::make_tuple(pixelPosRow, pixelPosCol);
}