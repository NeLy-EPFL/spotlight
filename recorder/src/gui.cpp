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
      serialPort_(new QSerialPort(this))
{
    // Configure serial port
    serialPortName_ = getSerialPortName();
    serialPort_.setPortName(QString::fromStdString(serialPortName_));
    serialPort_.setBaudRate(ARDUINO_BAUD_RATE_Q_ENUM);
    serialPort_.setDataBits(QSerialPort::Data8);
    serialPort_.setParity(QSerialPort::NoParity);
    serialPort_.setStopBits(QSerialPort::OneStop);
    serialPort_.setFlowControl(QSerialPort::NoFlowControl);

    bool serialPortOpened = serialPort_.open(QIODevice::ReadWrite);
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
    behaviorFPSSpinBox_ = new QSpinBox(this);
    behaviorFPSSpinBox_->setRange(1, 1000);
    behaviorFPSSpinBox_->setValue(BEHAVIOR_CAMERA_DEFAULT_FPS);
    QHBoxLayout *behaviorFPSLayout = new QHBoxLayout();
    behaviorFPSLayout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behaviorFPSLayout->addWidget(behaviorFPSSpinBox_);

    // Behavior exposure time widget
    behaviorExposureTimeSpinBox_ = new QDoubleSpinBox(this);
    behaviorExposureTimeSpinBox_->setRange(0.001, 1000.0);
    behaviorExposureTimeSpinBox_->setValue(
        BEHAVIOR_CAMERA_DEFAULT_EXPOSURE_TIME_MICROSECS / 1000.0);
    QHBoxLayout *behaviorExposureTimeLayout = new QHBoxLayout();
    behaviorExposureTimeLayout->addWidget(
        new QLabel("Behavior exposure time (ms)"));
    behaviorExposureTimeLayout->addWidget(behaviorExposureTimeSpinBox_);

    // Save directory widget
    directoryLineEdit_ = new QLineEdit(this);
    directoryLineEdit_->setText(saveDirectory.c_str());
    QPushButton *browseButton = new QPushButton("Browse", this);

    QHBoxLayout *directoryLayout = new QHBoxLayout();
    directoryLayout->addWidget(new QLabel("Save Directory"));
    directoryLayout->addWidget(directoryLineEdit_);
    directoryLayout->addWidget(browseButton);

    connect(browseButton, &QPushButton::clicked, this, &Gui::browseDirectory);

    // Live display widget
    behaviorImageDisplayLabel_ = new QLabel(this);
    behaviorImageDisplayLabel_->setFixedSize(
        GUI_BEHAVIOR_CAMERA_PREVIEW_WIDTH,
        GUI_BEHAVIOR_CAMERA_PREVIEW_HEIGHT);

    // Record and stop buttons
    recordButton_ = new QPushButton("Record", this);
    stopButton_ = new QPushButton("Stop", this);
    stopButton_->setEnabled(false); // initially disabled
    QHBoxLayout *recordStopButtonsLayout = new QHBoxLayout();
    recordStopButtonsLayout->addWidget(recordButton_);
    recordStopButtonsLayout->addWidget(stopButton_);

    // Add timer to update image display
    imageDisplayTimer_ = new QTimer(this);
    connect(imageDisplayTimer_,
            &QTimer::timeout,
            this,
            &Gui::updateImageDisplay);
    imageDisplayTimer_->start(1000 / BEHAVIOR_CAMERA_STREAMING_FPS);
    connect(recordButton_, &QPushButton::clicked, this, &Gui::startRecording);
    connect(stopButton_, &QPushButton::clicked, this, &Gui::stopRecording);

    // Arrange layout
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addLayout(behaviorFPSLayout);
    layout->addLayout(behaviorExposureTimeLayout);
    layout->addLayout(directoryLayout);
    layout->addWidget(behaviorImageDisplayLabel_);
    layout->addLayout(recordStopButtonsLayout);
    setLayout(layout);

    // Initialy stream images only, don't save
    stopRecording();
}

void Gui::startRecording()
{
    recordButton_->setEnabled(false);
    stopButton_->setEnabled(true);

    int recordingFPS = behaviorFPSSpinBox_->value();
    int recordingExposureTimeMicrosecs =
        behaviorExposureTimeSpinBox_->value() * 1000;

    runRecordingStartProcedure(
        serialPort_, recordingFPS, recordingExposureTimeMicrosecs);
}

void Gui::stopRecording()
{
    recordButton_->setEnabled(true);
    stopButton_->setEnabled(false);

    int recordingExposureTimeMicrosecs =
        behaviorExposureTimeSpinBox_->value() * 1000;

    runRecordingStopProcedure(serialPort_, recordingExposureTimeMicrosecs);
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
        directoryLineEdit_->setText(dir);
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
                         .scaled(behaviorImageDisplayLabel_->size(),
                                 Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
    behaviorImageDisplayLabel_->setPixmap(pixmap);
}