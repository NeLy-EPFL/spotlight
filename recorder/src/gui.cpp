#include "gui.hpp"

cv::Mat getLatestFrame()
{
    {
        std::lock_guard<std::mutex> lock(latestFrameMutex);
        if (latestFrameData.imagePtr == nullptr)
        {
            return cv::Mat();
        }
        cv::Mat latestFrameImage = latestFrameData.imagePtr->clone();
        return latestFrameImage;
    }
}

QImage cvMatToQImage(const cv::Mat &mat)
{
    if (mat.empty())
    {
        return QImage();
    }
    return QImage(mat.data,
                  mat.cols,
                  mat.rows,
                  mat.step,
                  QImage::Format_Grayscale8);
}

Gui::Gui(QWidget *parent)
    : QWidget(parent),
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

    bool serialPortOpened = serialPort.open(QIODevice::ReadWrite);
    if (serialPortOpened)
    {
        spdlog::info("Serial port opened successfully.");
    }
    else
    {
        spdlog::error("Failed to open serial port.");
        throw std::runtime_error("Failed to open serial port.");
    }

    // Behavior FPS widget
    behaviorFPSSpinBox = new QSpinBox(this);
    behaviorFPSSpinBox->setRange(1, 1000);
    behaviorFPSSpinBox->setValue(BEHAVIOR_CAMERA_DEFAULT_FPS);
    QHBoxLayout *behaviorFPSLayout = new QHBoxLayout();
    behaviorFPSLayout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behaviorFPSLayout->addWidget(behaviorFPSSpinBox);

    // Behavior exposure time widget
    behaviorExposureTimeSpinBox = new QDoubleSpinBox(this);
    behaviorExposureTimeSpinBox->setRange(0.001, 1000.0);
    behaviorExposureTimeSpinBox->setValue(
        BEHAVIOR_CAMERA_DEFAULT_EXPOSURE_TIME_MICROSECS / 1000.0);
    QHBoxLayout *behaviorExposureTimeLayout = new QHBoxLayout();
    behaviorExposureTimeLayout->addWidget(
        new QLabel("Behavior exposure time (ms)"));
    behaviorExposureTimeLayout->addWidget(behaviorExposureTimeSpinBox);

    // Save directory widget
    directoryLineEdit = new QLineEdit(this);
    directoryLineEdit->setText(saveDirectory.c_str());
    QPushButton *browseButton = new QPushButton("Browse", this);

    QHBoxLayout *directoryLayout = new QHBoxLayout();
    directoryLayout->addWidget(new QLabel("Save Directory"));
    directoryLayout->addWidget(directoryLineEdit);
    directoryLayout->addWidget(browseButton);

    connect(browseButton, &QPushButton::clicked, this, &Gui::browseDirectory);

    // Live display widget
    behaviorImageDisplayLabel = new QLabel(this);
    behaviorImageDisplayLabel->setFixedSize(
        GUI_BEHAVIOR_CAMERA_PREVIEW_WIDTH,
        GUI_BEHAVIOR_CAMERA_PREVIEW_HEIGHT);

    // Record and stop buttons
    recordButton = new QPushButton("Record", this);
    stopButton = new QPushButton("Stop", this);
    stopButton->setEnabled(false); // initially disabled
    QHBoxLayout *recordStopButtonsLayout = new QHBoxLayout();
    recordStopButtonsLayout->addWidget(recordButton);
    recordStopButtonsLayout->addWidget(stopButton);

    // Add timer to update image display
    imageDisplayTimer = new QTimer(this);
    connect(imageDisplayTimer,
            &QTimer::timeout,
            this,
            &Gui::updateImageDisplay);
    imageDisplayTimer->start(1000 / BEHAVIOR_CAMERA_STREAMING_FPS);
    connect(recordButton, &QPushButton::clicked, this, &Gui::startRecording);
    connect(stopButton, &QPushButton::clicked, this, &Gui::stopRecording);

    // Arrange layout
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addLayout(behaviorFPSLayout);
    layout->addLayout(behaviorExposureTimeLayout);
    layout->addLayout(directoryLayout);
    layout->addWidget(behaviorImageDisplayLabel);
    layout->addLayout(recordStopButtonsLayout);
    setLayout(layout);

    // Initialy stream images only, don't save
    stopRecording();
}

void Gui::startRecording()
{
    recordButton->setEnabled(false);
    stopButton->setEnabled(true);

    int recordingFPS = behaviorFPSSpinBox->value();
    int recordingExposureTimeMicrosecs =
        behaviorExposureTimeSpinBox->value() * 1000;

    runRecordingStartProcedure(
        serialPort, recordingFPS, recordingExposureTimeMicrosecs);
}

void Gui::stopRecording()
{
    recordButton->setEnabled(true);
    stopButton->setEnabled(false);

    int recordingExposureTimeMicrosecs =
        behaviorExposureTimeSpinBox->value() * 1000;

    runRecordingStopProcedure(serialPort, recordingExposureTimeMicrosecs);
}

void Gui::closeEvent(QCloseEvent *event)
{
    spdlog::info("User is closing GUI window. Quitting gracefully.");
    event->accept();
    quitProgram();
}

void Gui::browseDirectory()
{
    QString dir = QFileDialog::getExistingDirectory(
        this,
        "Open Directory",
        QString::fromStdString(saveDirectory),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty())
    {
        directoryLineEdit->setText(dir);
        saveDirectory = dir.toStdString();
    }
    else
    {
        spdlog::error("Directory is an empty string; failed to open.");
    }
}

void Gui::updateImageDisplay()
{
    cv::Mat latestFrame = getLatestFrame();
    if (latestFrame.empty())
    {
        return;
    }
    QImage qImage = cvMatToQImage(latestFrame);
    QPixmap pixmap = QPixmap::fromImage(qImage)
                         .scaled(behaviorImageDisplayLabel->size(),
                                 Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
    behaviorImageDisplayLabel->setPixmap(pixmap);
}