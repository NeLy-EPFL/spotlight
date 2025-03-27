#include "ArduinoCommunication.hpp"

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QMutex>
#include <QQueue>
#include <QThread>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <iostream>

// Global variables for serial communication
static QSerialPort* g_serialPort = nullptr;
static QMutex g_serialMutex;
static QQueue<std::string> g_messageQueue;
static QMutex g_queueMutex;

bool initializeArduinoCommunication(const std::string& portName, int baudRate) {
    QMutexLocker locker(&g_serialMutex);
    
    // Clean up any existing serial port
    if (g_serialPort) {
        if (g_serialPort->isOpen()) {
            g_serialPort->close();
        }
        delete g_serialPort;
        g_serialPort = nullptr;
    }
    
    // Create and configure the serial port
    g_serialPort = new QSerialPort(QString::fromStdString(portName));
    g_serialPort->setBaudRate(baudRate);
    g_serialPort->setDataBits(QSerialPort::Data8);
    g_serialPort->setParity(QSerialPort::NoParity);
    g_serialPort->setStopBits(QSerialPort::OneStop);
    g_serialPort->setFlowControl(QSerialPort::NoFlowControl);
    
    // Open the port
    if (!g_serialPort->open(QIODevice::ReadWrite)) {
        std::cerr << "Failed to open serial port: " << portName
                  << " - Error: " << g_serialPort->errorString().toStdString() << std::endl;
        delete g_serialPort;
        g_serialPort = nullptr;
        return false;
    }
    
    std::cout << "Successfully connected to Arduino on " << portName << std::endl;
    return true;
}

void shutdownArduinoCommunication() {
    QMutexLocker locker(&g_serialMutex);
    
    if (g_serialPort) {
        if (g_serialPort->isOpen()) {
            g_serialPort->close();
        }
        delete g_serialPort;
        g_serialPort = nullptr;
    }
}

/**
 * Helper function to process incoming data from Arduino
 */
static void processIncomingData(const QString& data) {
    // Split the data by newlines
    QStringList lines = data.split('\n', Qt::SkipEmptyParts);
    
    for (const QString& line : lines) {
        // Check if it's a log message (starts with ">")
        if (line.startsWith('>')) {
            std::cout << "Arduino log: " << line.toStdString() << std::endl;
        } else {
            // Otherwise, add it to the message queue
            QMutexLocker queueLocker(&g_queueMutex);
            g_messageQueue.enqueue(line.toStdString());
        }
    }
}

/**
 * Helper function to check for and process any available data
 */
static void checkForIncomingData() {
    QMutexLocker locker(&g_serialMutex);
    
    if (!g_serialPort || !g_serialPort->isOpen()) {
        return;
    }
    
    if (g_serialPort->bytesAvailable() > 0) {
        QByteArray data = g_serialPort->readAll();
        processIncomingData(QString::fromUtf8(data));
    }
}

/**
 * Helper function to send a command and wait for ACK
 */
static int sendCommandAndWaitForAck(const std::string& command, const std::string& expectedAckPrefix, int timeoutUs) {
    {
        QMutexLocker locker(&g_serialMutex);
        
        if (!g_serialPort || !g_serialPort->isOpen()) {
            std::cerr << "Serial port not open" << std::endl;
            return 1;
        }
        
        // Clear any pending data
        g_serialPort->clear();
        
        // Send the command with newline
        QByteArray dataToSend = QByteArray::fromStdString(command + "\n");
        qint64 bytesWritten = g_serialPort->write(dataToSend);
        g_serialPort->flush();
        
        if (bytesWritten != dataToSend.size()) {
            std::cerr << "Failed to write all data to serial port" << std::endl;
            return 1;
        }
        
        std::cout << "Sent command: " << command << std::endl;
    }
    
    // Wait for the ACK response with timeout
    QElapsedTimer timer;
    timer.start();
    
    while (timer.elapsed() * 1000 < timeoutUs) {
        {
            QMutexLocker locker(&g_serialMutex);
            
            // Check for new data
            if (g_serialPort && g_serialPort->waitForReadyRead(1)) {
                QByteArray responseData = g_serialPort->readAll();
                processIncomingData(QString::fromUtf8(responseData));
            }
        }
        
        // Check the message queue for ACK
        {
            QMutexLocker queueLocker(&g_queueMutex);
            
            // Iterate through messages looking for ACK
            for (int i = 0; i < g_messageQueue.size(); i++) {
                const std::string& message = g_messageQueue[i];
                
                // Check if this message is the ACK we're waiting for
                if (message.find(expectedAckPrefix) == 0) {
                    // Remove this message from the queue
                    g_messageQueue.removeAt(i);
                    std::cout << "Received ACK: " << message << std::endl;
                    return 0;  // Success
                }
            }
        }
        
        // Process events and sleep a little to prevent CPU hogging
        QCoreApplication::processEvents();
        QThread::usleep(1000);
    }
    
    std::cerr << "Timeout waiting for ACK response" << std::endl;
    return 1;  // Timeout occurred
}

int setBehaviorPeriod(int behaviorPeriodUs, int timeoutUs) {
    std::string command = formatSetBehaviorPeriodCommand(behaviorPeriodUs);
    std::string ackPrefix = std::string(CMDSTR_SET_BEHAVIOR_PERIOD) + std::string(ACK_SUFFIX);
    return sendCommandAndWaitForAck(command, ackPrefix, timeoutUs);
}

int setSyncRatio(int syncRatio, int timeoutUs) {
    std::string command = formatSetSyncRatioCommand(syncRatio);
    std::string ackPrefix = std::string(CMDSTR_SET_SYNC_RATIO) + std::string(ACK_SUFFIX);
    return sendCommandAndWaitForAck(command, ackPrefix, timeoutUs);
}

int setBehaviorExposureTime(int exposureTimeUs, int timeoutUs) {
    std::string command = formatSetBehaviorExposureTimeCommand(exposureTimeUs);
    std::string ackPrefix = std::string(CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME) + std::string(ACK_SUFFIX);
    return sendCommandAndWaitForAck(command, ackPrefix, timeoutUs);
}

int setMuscleExposureTime(int exposureTimeUs, int timeoutUs) {
    std::string command = formatSetMuscleExposureTimeCommand(exposureTimeUs);
    std::string ackPrefix = std::string(CMDSTR_SET_MUSCLE_EXPOSURE_TIME) + std::string(ACK_SUFFIX);
    return sendCommandAndWaitForAck(command, ackPrefix, timeoutUs);
}

int startTriggering(ProtocolStep* protocolSteps, int timeoutUs) {
    // Convert the protocolSteps array to a vector for formatting
    std::vector<ProtocolStep> stepsVector;
    
    // Copy steps until we find one with isDone=true (or a null entry)
    for (int i = 0; protocolSteps[i].frameCount >= 0; i++) {
        stepsVector.push_back(protocolSteps[i]);
        
        // If this is the terminating step, stop adding more
        if (protocolSteps[i].isDone) {
            break;
        }
    }
    
    // Format the protocol string
    std::string protocolStr = formatEntireProtocol(stepsVector);
    
    // Create and send the command
    std::string command = formatStartTriggeringCommand(protocolStr);
    std::string ackPrefix = std::string(CMDSTR_START_TRIGGERING) + std::string(ACK_SUFFIX);
    
    return sendCommandAndWaitForAck(command, ackPrefix, timeoutUs);
}

int stopTriggering(int timeoutUs) {
    std::string command = formatStopTriggeringCommand();
    std::string ackPrefix = std::string(CMDSTR_STOP_TRIGGERING) + std::string(ACK_SUFFIX);
    return sendCommandAndWaitForAck(command, ackPrefix, timeoutUs);
}

bool incomingArduinoMessageAvailable() {
    // Check for new data first
    checkForIncomingData();
    
    // Check if there are any messages in the queue
    QMutexLocker queueLocker(&g_queueMutex);
    return !g_messageQueue.isEmpty();
}

std::string readArduinoMessageIfAvailable() {
    // Check for new data first
    checkForIncomingData();
    
    // Check the message queue
    QMutexLocker queueLocker(&g_queueMutex);
    
    if (g_messageQueue.isEmpty()) {
        return "";
    }
    
    return g_messageQueue.dequeue();
}

std::string waitForArduinoMessage(int timeoutUs) {
    QElapsedTimer timer;
    timer.start();
    
    while (timer.elapsed() * 1000 < timeoutUs) {
        // Check for and process new data
        {
            QMutexLocker locker(&g_serialMutex);
            
            if (g_serialPort && g_serialPort->isOpen()) {
                if (g_serialPort->waitForReadyRead(1)) {
                    QByteArray data = g_serialPort->readAll();
                    processIncomingData(QString::fromUtf8(data));
                }
            }
        }
        
        // Check if we have a message now
        {
            QMutexLocker queueLocker(&g_queueMutex);
            
            if (!g_messageQueue.isEmpty()) {
                return g_messageQueue.dequeue();
            }
        }
        
        // Process events and sleep a little to prevent CPU hogging
        QCoreApplication::processEvents();
        QThread::usleep(1000);
    }
    
    // Timeout occurred
    return "";
}