#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>

namespace asio = boost::asio;

class ArduinoSerial {
private:
    asio::io_context io;
    asio::serial_port serial;
    std::string port;
    unsigned int baud_rate;

public:
    ArduinoSerial(const std::string& port, unsigned int baud_rate)
        : serial(io), port(port), baud_rate(baud_rate) {}

    bool connect() {
        try {
            serial.open(port);
            
            // Configure serial port
            serial.set_option(asio::serial_port_base::baud_rate(baud_rate));
            serial.set_option(asio::serial_port_base::character_size(8));
            serial.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
            serial.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
            serial.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
            
            // Wait for Arduino to reset after connection
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            return true;
        } catch (const boost::system::system_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return false;
        }
    }

    void write(const std::string& message) {
        asio::write(serial, asio::buffer(message));
    }

    std::string read_line() {
        asio::streambuf buffer;
        asio::read_until(serial, buffer, '\n');
        
        std::string line;
        std::istream stream(&buffer);
        std::getline(stream, line);
        return line;
    }

    void close() {
        if (serial.is_open()) {
            serial.close();
        }
    }

    ~ArduinoSerial() {
        close();
    }
};

int main(int argc, char* argv[]) {
    // Default values
    std::string port = "/dev/ttyACM0";  // Default port for Arduino on Linux
    unsigned int baud_rate = 9600;       // Default baud rate

    // Override with command-line arguments if provided
    if (argc > 1) {
        port = argv[1];
    }
    if (argc > 2) {
        try {
            baud_rate = std::stoi(argv[2]);
        } catch (...) {
            std::cerr << "Invalid baud rate specified. Using default: " << baud_rate << std::endl;
        }
    }

    std::cout << "Arduino Hello World" << std::endl;
    std::cout << "Connecting to Arduino on " << port << " at " << baud_rate << " baud..." << std::endl;

    ArduinoSerial arduino(port, baud_rate);
    
    if (!arduino.connect()) {
        std::cerr << "Failed to connect to Arduino!" << std::endl;
        return 1;
    }
    
    std::cout << "Connected to Arduino. Sending 'Hello, Arduino!'" << std::endl;
    
    // Send message to Arduino
    arduino.write("Hello, Arduino!\n");
    
    // Read response from Arduino
    std::cout << "Waiting for response..." << std::endl;
    try {
        std::string response = arduino.read_line();
        std::cout << "Arduino says: " << response << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error reading from Arduino: " << e.what() << std::endl;
    }
    
    std::cout << "Closing connection..." << std::endl;
    arduino.close();
    
    return 0;
}