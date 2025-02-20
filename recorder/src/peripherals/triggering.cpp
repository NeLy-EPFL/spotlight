// #include "triggering.hpp"

// namespace
// {
//     void sendCommand(
//         boost::asio::serial_port *serialPort, const std::string &command)
//     {
//         if (serialPort && serialPort->is_open())
//         {
//             boost::system::error_code errCode;
//             boost::asio::write(
//                 *serialPort, boost::asio::buffer(command + "\n"), errCode);
//             if (errCode)
//             {
//                 spdlog::error("Error sending command: {}", errCode.message());
//             }
//             else
//             {
//                 std::cout << "Sent command: " << command << std::endl;
//             }
//         }
//     }
// }

// TriggerController::TriggerController(
//     const std::string &portName, unsigned int baudRate)
//     : serialPortPtr_(nullptr)
// {
//     try
//     {
//         serialPortPtr_ = new boost::asio::serial_port(ioContext_, portName);
//         serialPortPtr_->set_option(
//             boost::asio::serial_port_base::baud_rate(baudRate));
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << "Error opening serial port: " << e.what() << std::endl;
//     }
// }

// TriggerController::~TriggerController()
// {
//     if (serialPortPtr_)
//     {
//         if (serialPortPtr_->is_open())
//         {
//             serialPortPtr_->close();
//         }
//         delete serialPortPtr_;
//     }
// }

// void TriggerController::startRecording(int recordingFPS)
// {
//     if (recordingFPS <= 0)
//     {
//         std::cerr << "Invalid recording FPS: " << recordingFPS << std::endl;
//         return;
//     }

//     // Send a command string: eg. "record 400" if it's 400 FPS
//     std::ostringstream oss;
//     oss << "record " << recordingFPS;
//     sendCommand(serialPortPtr_, oss.str());

//     std::ostringstream oss;
//     oss << "start " << recordingFPS;
//     sendCommand(serialPortPtr_, oss.str());
// }

// void TriggerController::stopRecording()
// {
//     sendCommand(serialPortPtr_, "stop");
// }