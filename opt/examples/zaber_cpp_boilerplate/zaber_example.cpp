#include <iostream>
#include <vector>
#include <zaber/motion/ascii.h>

using namespace zaber::motion;
using namespace zaber::motion::ascii;

int main() {
    Library::checkVersion();

    Connection connection = Connection::openSerialPort("/dev/ttyACM0");
    connection.enableAlerts();

    std::vector<Device> deviceList = connection.detectDevices();
    std::cout << "Found " << deviceList.size() << " devices" << std::endl;

    Device device = deviceList[0];
    int numAxis = device.getAxisCount();
    std::cout << "Found " << numAxis << " axes" << std::endl;

    for (int i = 0; i < numAxis; ++i) {
        Axis axis = device.getAxis(i + 1);
        if (!axis.isHomed()) {
            axis.home();
        }

        // Move to 10mm
        axis.moveAbsolute(10, Units::LENGTH_MILLIMETRES);

        // Move by an additional 5mm
        axis.moveRelative(5, Units::LENGTH_MILLIMETRES);
    }

    return 0;
}