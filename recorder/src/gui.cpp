#include "gui.hpp"

namespace
{
    void sendCommand(QSerialPort &serialPort, const QString &command)
    {
        if (serialPort.isOpen())
        {
            serialPort.write(command.toUtf8() + '\n');
        }
        else
        {
            spdlog::error("Failed to send command; serial port not open.");
        }
    }
}

Gui::Gui(std::shared_ptr<std::atomic<bool>> isSavingData,
         std::shared_ptr<std::atomic<bool>> toQuit,
         QWidget *parent)
    : QWidget(parent),
      isSavingData(isSavingData),
      toQuit(toQuit),
      directory("./images"),
      serialPort(new QSerialPort(this))
{
    // Configure serial port
    serialPortName = getSerialPortName();
    serialPort.setPortName(QString::fromStdString(serialPortName));
    serialPort.setBaudRate(QSerialPort::Baud9600);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);

    bool serialPortOpened = serialPort.open(QIODevice::WriteOnly);
    if (serialPortOpened)
    {
        spdlog::info("Serial port opened successfully.");
    }
    else
    {
        spdlog::error("Failed to open serial port.");
        throw std::runtime_error("Failed to open serial port.");
    }

    recordButton = new QPushButton("Record", this);
    stopButton = new QPushButton("Stop", this);
    stopButton->setEnabled(false); // initially disabled

    behaviorFPSSpinBox = new QSpinBox(this);
    behaviorFPSSpinBox->setRange(1, 1000);
    behaviorFPSSpinBox->setValue(100);

    behaviorExposureTimeSpinBox = new QDoubleSpinBox(this);
    behaviorExposureTimeSpinBox->setRange(0.001, 1000.0);
    behaviorExposureTimeSpinBox->setValue(1);

    stopRecording(); // initialy stream images only, don't save

    QVBoxLayout *layout = new QVBoxLayout(this);
    QHBoxLayout *behaviorFPSLayout = new QHBoxLayout();
    behaviorFPSLayout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behaviorFPSLayout->addWidget(behaviorFPSSpinBox);
    layout->addLayout(behaviorFPSLayout);

    QHBoxLayout *behaviorExposureTimeLayout = new QHBoxLayout();
    behaviorExposureTimeLayout->addWidget(
        new QLabel("Behavior exposure time (ms)"));
    behaviorExposureTimeLayout->addWidget(behaviorExposureTimeSpinBox);
    layout->addLayout(behaviorExposureTimeLayout);

    layout->addWidget(recordButton);
    layout->addWidget(stopButton);

    connect(recordButton, &QPushButton::clicked, this, &Gui::startRecording);
    connect(stopButton, &QPushButton::clicked, this, &Gui::stopRecording);

    setLayout(layout);
}

void Gui::startRecording()
{
    *isSavingData = true;

    CameraAcquisitionConfig cameraAcquisitionConfig(
        CameraAcquisitionMode::RECORD,
        behaviorFPSSpinBox->value(),
        behaviorExposureTimeSpinBox->value() * 1000); // convert to microseconds
    std::string commandString = cameraAcquisitionConfig.toCommandString();
    sendCommand(serialPort, QString(commandString.c_str()));

    recordButton->setEnabled(false);
    stopButton->setEnabled(true);
}

void Gui::stopRecording()
{
    *isSavingData = false;

    CameraAcquisitionConfig cameraAcquisitionConfig(
        CameraAcquisitionMode::STREAM,
        BEHAVIOR_CAMERA_STREAMING_FPS,
        behaviorExposureTimeSpinBox->value() * 1000); // convert to microseconds
    std::string commandString = cameraAcquisitionConfig.toCommandString();
    sendCommand(serialPort, QString(commandString.c_str()));

    recordButton->setEnabled(true);
    stopButton->setEnabled(false);
}
