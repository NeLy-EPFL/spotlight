#include "gui.hpp"

namespace
{
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
}

MotionControlWidget::MotionControlWidget(QWidget *parent)
    : QWidget(parent)
{
    connect(&timer_,
            &QTimer::timeout,
            this,
            QOverload<>::of(&MotionControlWidget::update));
    timer_.start(1000 / GUI_MOTION_STAGE_PREVIEW_FREQUENCY_HZ); // in ms
    int guiMotionStagePreviewWidth = calculateBehaviorCameraPreviewWidth(
        GUI_MOTION_STAGE_PREVIEW_HEIGHT,
        MOTION_STAGE_X_MAX_PHYSICAL_MM - MOTION_STAGE_X_MIN_PHYSICAL_MM,
        MOTION_STAGE_Y_MAX_PHYSICAL_MM - MOTION_STAGE_Y_MIN_PHYSICAL_MM);
    setFixedSize(guiMotionStagePreviewWidth,
                 GUI_MOTION_STAGE_PREVIEW_HEIGHT);
}

MotionControlWidget::~MotionControlWidget()
{
    timer_.stop();
}

void MotionControlWidget::paintEvent(QPaintEvent *event)
{
    if (!motionControlHandlerReady.load())
    {
        return;
    }

    Q_UNUSED(event);
    QPainter painter(this);

    // Draw the light gray rectangle representing the stage boundaries
    painter.fillRect(rect(), QColor(220, 220, 220));
    painter.setPen(Qt::black);
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    // Calculate where to draw the red dot representing the stage position
    MotionStagePosition currStagePosition = getCurrentMotionStagePosition();
    float physicalX = currStagePosition.xPosMm;
    float physicalY = currStagePosition.yPosMm;
    int pixelX = mapToPixelX(physicalX);
    int pixelY = mapToPixelY(physicalY);

    // Draw the red dot
    painter.setPen(Qt::red);
    painter.setBrush(Qt::red);
    int dotDiameter = 10;
    painter.drawEllipse(pixelX - dotDiameter / 2,
                        pixelY - dotDiameter / 2,
                        dotDiameter,
                        dotDiameter);

    // Draw coordinate labels
    painter.setPen(Qt::black);
    int minFieldWidth = 0;
    int precision = 2;
    int textPositionX = 10;
    int textPositionY = 20;
    painter.drawText(
        textPositionX, textPositionY,
        QString("(%1, %2) mm")
            .arg(physicalX, minFieldWidth, 'f', precision)
            .arg(physicalY, minFieldWidth, 'f', precision));
}

void MotionControlWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        float stageX = mapToStageX(event->position().x());
        float stageY = mapToStageY(event->position().y());
        setTargetMotionStagePosition({stageX, stageY, ABSOLUTE});
    }
}

int MotionControlWidget::mapToPixelX(float x) const
{
    return (x - minXAbsoluteMm_) / (maxXAbsoluteMm_ - minXAbsoluteMm_) *
               width() +
           0.5;
}

int MotionControlWidget::mapToPixelY(float y) const
{
    return (y - minYAbsoluteMm_) / (maxYAbsoluteMm_ - minYAbsoluteMm_) *
               height() +
           0.5;
}

float MotionControlWidget::mapToStageX(int x) const
{
    return x / static_cast<float>(width()) * (maxXAbsoluteMm_ - minXAbsoluteMm_) +
           minXAbsoluteMm_;
}

float MotionControlWidget::mapToStageY(int y) const
{
    return y / static_cast<float>(height()) * (maxYAbsoluteMm_ - minYAbsoluteMm_) +
           minYAbsoluteMm_;
}

MainGUIWindow::MainGUIWindow(QWidget *parent)
    : QWidget(parent)
{
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

    connect(browseButton,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::browseDirectory);

    // Live display widget
    behaviorImageDisplayLabel_ = new QLabel(this);
    behaviorImageDisplayLabel_->setFixedSize(
        GUI_BEHAVIOR_CAMERA_PREVIEW_WIDTH,
        GUI_BEHAVIOR_CAMERA_PREVIEW_HEIGHT);

    motionControlWidget_ = new MotionControlWidget(this);

    // Record and stop buttons
    recordButton_ = new QPushButton("Record", this);
    stopButton_ = new QPushButton("Stop", this);
    calibrationScanButton_ = new QPushButton("Calibration Scan", this);
    stopButton_->setEnabled(false); // initially disabled
    QHBoxLayout *recordStopButtonsLayout = new QHBoxLayout();
    recordStopButtonsLayout->addWidget(recordButton_);
    recordStopButtonsLayout->addWidget(stopButton_);
    recordStopButtonsLayout->addWidget(calibrationScanButton_);

    // Add timer to update image display
    imageDisplayTimer_ = new QTimer(this);
    connect(imageDisplayTimer_,
            &QTimer::timeout,
            this,
            &MainGUIWindow::updateImageDisplay);
    imageDisplayTimer_->start(1000 / BEHAVIOR_CAMERA_STREAMING_FPS);
    connect(recordButton_,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::startRecording);
    connect(stopButton_,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::stopRecording);
    connect(calibrationScanButton_,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::doCalibrationScan);

    // Arrange layout
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addLayout(behaviorFPSLayout);
    layout->addLayout(behaviorExposureTimeLayout);
    layout->addLayout(directoryLayout);
    layout->addWidget(behaviorImageDisplayLabel_);
    layout->addWidget(motionControlWidget_);
    layout->addLayout(recordStopButtonsLayout);
    setLayout(layout);

    // Initialy stream images only, don't save
    stopRecording();
}

void MainGUIWindow::startRecording()
{
    if (!behaviorCameraReady.load())
    {
        spdlog::error("Behavior camera not ready. Cannot start recording.");
        // Make a pop-up error window
        QMessageBox::critical(
            this,
            "Error",
            "Behavior camera not ready yet. Please wait 10 seconds. If the "
            "error persists, something has gone wrong. Check logs for info.");
        return;
    }

    recordButton_->setEnabled(false);
    stopButton_->setEnabled(true);
    calibrationScanButton_->setEnabled(false);

    int recordingFPS = behaviorFPSSpinBox_->value();
    int recordingExposureTimeMicrosecs =
        behaviorExposureTimeSpinBox_->value() * 1000;

    triggerController->startRecording(
        recordingFPS, recordingExposureTimeMicrosecs);
}

void MainGUIWindow::stopRecording()
{
    recordButton_->setEnabled(true);
    stopButton_->setEnabled(false);
    calibrationScanButton_->setEnabled(true);

    int recordingExposureTimeMicrosecs =
        behaviorExposureTimeSpinBox_->value() * 1000;

    triggerController->stopRecording(recordingExposureTimeMicrosecs);
}

void MainGUIWindow::doCalibrationScan()
{
    calibrationScanButton_->setEnabled(false);
    recordButton_->setEnabled(false);

    int currentStreamingExposureTimeMicrosecs =
        behaviorExposureTimeSpinBox_->value() * 1000;

    // Run the calibration scan in a separate thread to avoid freezing the GUI
    std::thread([this, currentStreamingExposureTimeMicrosecs]() {
        isRunningCalibrationScan_.store(true);
        spdlog::info("Starting calibration scan procedure in the background.");
        runCalibrationScanProcedure(currentStreamingExposureTimeMicrosecs);

        // Re-enable buttons in the GUI thread
        QMetaObject::invokeMethod(this, [this]() {
            calibrationScanButton_->setEnabled(true);
            recordButton_->setEnabled(true);
        });

        isRunningCalibrationScan_.store(false);
    }).detach();
}

bool MainGUIWindow::canQuitGracefully()
{
    return !isRunningCalibrationScan_.load();
}

void MainGUIWindow::closeEvent(QCloseEvent *event)
{
    spdlog::info("User is closing GUI window. Quitting gracefully.");
    if (!quitProgram()){
        event->ignore();
        return;
    }
    event->accept();
}

void MainGUIWindow::browseDirectory()
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

void MainGUIWindow::updateImageDisplay()
{
    cv::Mat latestFrame = getLatestFrame();
    if (latestFrame.empty())
    {
        return;
    }
    cv::Mat correctedFrame = correctImageRotationAndFlip(latestFrame);
    QImage qImage = cvMatToQImage(correctedFrame);
    QPixmap pixmap = QPixmap::fromImage(qImage)
                         .scaled(behaviorImageDisplayLabel_->size(),
                                 Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
    behaviorImageDisplayLabel_->setPixmap(pixmap);
}