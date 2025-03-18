#include "gui.hpp"

namespace
{
    cv::Mat getLatestFrame()
    {
        {
            std::lock_guard<std::mutex> lock(latestFrameMutex);
            cv::Mat latestFrameImage = latestFrameData.image;
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

MotionControlWidget::MotionControlWidget(
    const RecorderConfig &recorderConfig,
    TrackingControlState &trackingControlState,
    QWidget *parent)
    : QWidget(parent),
      trackingControlState_(trackingControlState)
{
    minXAbsoluteMm_ = recorderConfig.getParameter<double>("motion_control",
                                                          "x_min_mm");
    maxXAbsoluteMm_ = recorderConfig.getParameter<double>("motion_control",
                                                          "x_max_mm");
    minYAbsoluteMm_ = recorderConfig.getParameter<double>("motion_control",
                                                          "y_min_mm");
    maxYAbsoluteMm_ = recorderConfig.getParameter<double>("motion_control",
                                                          "y_max_mm");

    int guiMotionStagePreviewUpdateFreq = recorderConfig.getParameter<int>(
        "gui", "motion_stage_preview_update_frequency_hz");
    int guiMotionStagePreviewHeight = recorderConfig.getParameter<int>(
        "gui", "motion_stage_preview_height");

    connect(&timer_,
            &QTimer::timeout,
            this,
            QOverload<>::of(&MotionControlWidget::update));
    timer_.start(1000 / guiMotionStagePreviewUpdateFreq); // in ms
    int guiMotionStagePreviewWidth = calculateBehaviorCameraPreviewWidth(
        guiMotionStagePreviewHeight,
        maxXAbsoluteMm_ - minXAbsoluteMm_,
        maxYAbsoluteMm_ - minYAbsoluteMm_);
    setFixedSize(guiMotionStagePreviewWidth, guiMotionStagePreviewHeight);
}

MotionControlWidget::~MotionControlWidget()
{
    timer_.stop();
}

void MotionControlWidget::paintEvent(QPaintEvent *event)
{
    if (!trackingControlState_.motionControlHandlerReady.load())
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
    MotionStagePosition currStagePosition;
    {
        std::lock_guard<std::mutex> lock(
            trackingControlState_.latestMotionStagePositionMutex);
        currStagePosition = trackingControlState_.latestMotionStagePosition;
    }
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
        spdlog::debug("Clicked at ({}, {})", stageX, stageY);
        trackingControlState_.overridingPosX.store(stageX);
        trackingControlState_.overridingPosY.store(stageY);
        trackingControlState_.shouldOverrideTracking.store(true);
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

MainGUIWindow::MainGUIWindow(const RecorderConfig &recorderConfig,
                             BehaviorRecordingState &behaviorRecordingState,
                             TrackingControlState &trackingControlState,
                             CalibrationParams &behaviorCamCalibrationParams,
                             QWidget *parent)
    : QWidget(parent),
      recorderConfig_(recorderConfig),
      behaviorRecordingState_(behaviorRecordingState),
      trackingControlState_(trackingControlState),
      behaviorCamCalibrationParams_(behaviorCamCalibrationParams)
{
    // Behavior FPS widget
    behaviorFPSSpinBox_ = new QSpinBox(this);
    behaviorFPSSpinBox_->setRange(1, 1000);
    int behaviorCameraDefaultRecordingFrameRate =
        recorderConfig.getParameter<int>("behavior_camera",
                                         "default_recording_fps");
    behaviorFPSSpinBox_->setValue(behaviorCameraDefaultRecordingFrameRate);
    QHBoxLayout *behaviorFPSLayout = new QHBoxLayout();
    behaviorFPSLayout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behaviorFPSLayout->addWidget(behaviorFPSSpinBox_);

    // Behavior exposure time widget
    behaviorExposureTimeSpinBox_ = new QDoubleSpinBox(this);
    behaviorExposureTimeSpinBox_->setRange(0.001, 1000.0);
    int behaviorCameraDefaultExposureTimeUs = recorderConfig.getParameter<int>(
        "behavior_camera", "default_exposure_time_us");
    behaviorExposureTimeSpinBox_->setValue(
        behaviorCameraDefaultExposureTimeUs / 1000.0);
    QHBoxLayout *behaviorExposureTimeLayout = new QHBoxLayout();
    behaviorExposureTimeLayout->addWidget(
        new QLabel("Behavior exposure time (ms)"));
    behaviorExposureTimeLayout->addWidget(behaviorExposureTimeSpinBox_);

    // Save directory widget
    directoryLineEdit_ = new QLineEdit(this);
    directoryLineEdit_->setText(saveDirectory.c_str());
    connect(directoryLineEdit_,
            &QLineEdit::textChanged,
            this,
            [this](const QString &text)
            { spdlog::debug("saveDirectory changed to {}", text.toStdString());
                saveDirectory = text.toStdString(); });
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
    int behaviorCameraPreviewWidth = recorderConfig.getParameter<int>(
        "gui", "behavior_camera_preview_width");
    int behaviorCameraPreviewHeight = recorderConfig.getParameter<int>(
        "gui", "behavior_camera_preview_height");
    behaviorImageDisplayLabel_->setFixedSize(behaviorCameraPreviewWidth,
                                             behaviorCameraPreviewHeight);

    motionControlWidget_ = new MotionControlWidget(recorderConfig,
                                                   trackingControlState,
                                                   this);

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
            &MainGUIWindow::updateImageDisplay);
    int behaviorCameraStreamingFrameRate = recorderConfig.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");
    imageDisplayTimer_->start(1000 / behaviorCameraStreamingFrameRate);
    connect(recordButton_,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::startRecording);
    connect(stopButton_,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::stopRecording);

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
    if (!behaviorRecordingState_.behaviorCamera ||
        !behaviorRecordingState_.behaviorCamera->isReady())
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

    int recordingExposureTimeMicrosecs =
        behaviorExposureTimeSpinBox_->value() * 1000;

    triggerController->stopRecording(recordingExposureTimeMicrosecs);
}

void MainGUIWindow::closeEvent(QCloseEvent *event)
{
    spdlog::info("User is closing GUI window. Quitting gracefully.");
    if (!quitProgram())
    {
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

cv::Mat addCornerMarker(cv::Mat image,
                        MotionStagePosition stagePosition,
                        CalibrationParams &behaviorCamCalibrationParams)
{
    cv::Mat imageForDisplay = image.clone();
    assert(imageForDisplay.size() == image.size());

    std::vector<std::tuple<double, double>> cornerPositions = {
        {0, 0}, {48, 0}, {48, 72}, {0, 72}};
    std::vector<cv::Point> pixelPoints;
    for (auto [x, y] : cornerPositions)
    {
        int pixelRow, pixelCol;
        std::tie(pixelRow, pixelCol) =
            behaviorCamCalibrationParams.stagePosAndPhysicalPosToPixelPos(
                stagePosition.xPosMm, stagePosition.yPosMm, x, y);
        pixelPoints.emplace_back(pixelRow, pixelCol);
        cv::circle(imageForDisplay,
                   cv::Point(pixelCol, pixelRow),
                   5,
                   cv::Scalar(255, 255, 255),
                   -1);

        // if (0 <= pixelCol && pixelCol < imageForDisplay.cols &&
        //     0 <= pixelRow && pixelRow < imageForDisplay.rows)
        // {
        //     spdlog::info(
        //         "Corner marker drawn: "
        //         "(stagePos=({:.2f}, {:.2f}), physicalPos=({:.2f}, {:.2f})) "
        //         "-> pixelPos(r{}, c{})",
        //         stagePosition.xPosMm,
        //         stagePosition.yPosMm,
        //         x,
        //         y,
        //         pixelRow,
        //         pixelCol);
        // }
    }
    for (size_t i = 0; i < pixelPoints.size(); ++i)
    {
        cv::line(imageForDisplay,
                 pixelPoints[i],
                 pixelPoints[(i + 1) % pixelPoints.size()],
                 cv::Scalar(255, 0, 0), 2);
    }

    return imageForDisplay;
}

void MainGUIWindow::updateImageDisplay()
{
    cv::Mat latestFrame = getLatestFrame();
    if (latestFrame.empty())
    {
        return;
    }
    cv::Mat correctedFrame = correctImageRotationAndFlip(latestFrame);

    MotionStagePosition myStagePosition;
    {
        std::lock_guard<std::mutex> lock(
            trackingControlState_.latestMotionStagePositionMutex);
        myStagePosition = trackingControlState_.latestMotionStagePosition;
    }

    cv::Mat maskedImage = blackoutOutside(
        correctedFrame,
        myStagePosition,
        behaviorCamCalibrationParams_,
        recorderConfig_);

    cv::Mat imageForDisplay = addCornerMarker(maskedImage,
                                              myStagePosition,
                                              behaviorCamCalibrationParams_);

    QImage qImage = cvMatToQImage(imageForDisplay);
    QPixmap pixmap = QPixmap::fromImage(qImage)
                         .scaled(behaviorImageDisplayLabel_->size(),
                                 Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
    behaviorImageDisplayLabel_->setPixmap(pixmap);
}