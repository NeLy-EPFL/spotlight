// #ifndef RECORDING_CONTROLLER_HPP
// #define RECORDING_CONTROLLER_HPP

// #include <iostream>
// #include <sstream>
// #include <string>

// #include <boost/asio.hpp>
// #include <spdlog/spdlog.h>

// class TriggerController
// {
// public:
//     explicit TriggerController(const std::string &port, unsigned int baudRate);
//     ~TriggerController();

//     void startRecording(int recordingFPS);
//     void stopRecording();

// private:
//     boost::asio::io_context ioContext_;
//     boost::asio::serial_port *serialPortPtr_;
// };

// #endif // RECORDING_CONTROLLER_HPP