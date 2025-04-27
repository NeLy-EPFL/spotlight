#include "gui.hpp"

namespace
{
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
    std::shared_ptr<TrackingControlState> trackingControlState,
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
    if (!trackingControlState_->motionControlHandlerReady.load())
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
            trackingControlState_->latestMotionStagePositionMutex);
        currStagePosition = trackingControlState_->latestMotionStagePosition;
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
        trackingControlState_->overridingPosX.store(stageX);
        trackingControlState_->overridingPosY.store(stageY);
        trackingControlState_->shouldOverrideTracking.store(true);
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

MainGUIWindow::MainGUIWindow(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<TrackingControlState> trackingControlState,
    CalibrationParams &behaviorCamCalibrationParams,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ArduinoCommunication> arduinoCommunication,
    std::shared_ptr<ProgramState> programState,
    std::shared_ptr<ProgrammedStop> programmedRecordingStop,
    QWidget *parent)
    : QWidget(parent),
      recorderConfig_(recorderConfig),
      behaviorRecordingState_(behaviorRecordingState),
      trackingControlState_(trackingControlState),
      behaviorCamCalibrationParams_(behaviorCamCalibrationParams),
      saveDirectory_(saveDirectory),
      arduinoCommunication_(arduinoCommunication),
      programState_(programState),
      programmedRecordingStop_(programmedRecordingStop)
{
    streamingBehaviorFPS_ = recorderConfig.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");
    streamingSyncRatio_ = recorderConfig.getParameter<int>(
        "muscle_camera", "streaming_sync_ratio");

    // Behavior FPS widget
    behaviorFPSSpinBox_ = new QSpinBox(this);
    behaviorFPSSpinBox_->setRange(1, 1000);
    int behaviorCameraDefaultRecordingFrameRate =
        recorderConfig.getParameter<int>("behavior_camera",
                                         "default_recording_fps");
    behaviorFPSSpinBox_->setValue(behaviorCameraDefaultRecordingFrameRate);
    // Don't connect to ArduinoCommunication! This value is only used during
    // recording. When streaming, the sync ratio is always 1 and this field is
    // ignored.
    QHBoxLayout *behaviorFPSLayout = new QHBoxLayout();
    behaviorFPSLayout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behaviorFPSLayout->addWidget(behaviorFPSSpinBox_);

    // Behavior-muscle synchronization ratio
    syncRatioSpinBox_ = new QSpinBox(this);
    syncRatioSpinBox_->setRange(1, 10);
    int syncRatio = recorderConfig.getParameter<int>(
        "muscle_camera", "default_recording_sync_ratio");
    syncRatioSpinBox_->setValue(syncRatio);
    // Don't connect to ArduinoCommunication! This value is only used during
    // recording. When streaming, the sync ratio is always 1 and this field is
    // ignored.
    QHBoxLayout *syncRatioLayout = new QHBoxLayout();
    syncRatioLayout->addWidget(new QLabel("Behavior FPS : muscle FPS"));
    syncRatioLayout->addWidget(syncRatioSpinBox_);

    // Behavior exposure time widget
    behaviorExposureTimeSpinBox_ = new QDoubleSpinBox(this);
    behaviorExposureTimeSpinBox_->setRange(0.001, 1000.0);
    int behaviorCameraDefaultExposureTimeUs = recorderConfig.getParameter<int>(
        "behavior_camera", "default_exposure_time_us");
    behaviorExposureTimeSpinBox_->setValue(
        behaviorCameraDefaultExposureTimeUs / 1000.0);
    connect(behaviorExposureTimeSpinBox_,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this, arduinoCommunication](double value)
            { arduinoCommunication->setBehaviorExposureTime(value * 1000); });
    QHBoxLayout *behaviorExposureTimeLayout = new QHBoxLayout();
    behaviorExposureTimeLayout->addWidget(
        new QLabel("Behavior exposure time (ms)"));
    behaviorExposureTimeLayout->addWidget(behaviorExposureTimeSpinBox_);

    // Muscle exposure time widget
    muscleExposureTimeSpinBox_ = new QDoubleSpinBox(this);
    muscleExposureTimeSpinBox_->setRange(0.001, 1000.0);
    int muscleCameraDefaultExposureTimeUs = recorderConfig.getParameter<int>(
        "muscle_camera", "default_exposure_time_us");
    muscleExposureTimeSpinBox_->setValue(
        muscleCameraDefaultExposureTimeUs / 1000.0);
    connect(muscleExposureTimeSpinBox_,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this, arduinoCommunication](double value)
            { arduinoCommunication->setMuscleExposureTime(value * 1000); });
    QHBoxLayout *muscleExposureTimeLayout = new QHBoxLayout();
    muscleExposureTimeLayout->addWidget(
        new QLabel("Muscle exposure time (ms)"));
    muscleExposureTimeLayout->addWidget(muscleExposureTimeSpinBox_);

    // Experiment protocol
    // Experiment protocol widget
    QLabel *protocolLabel = new QLabel("Experiment protocol", this);
    experimentProtocol_ = new QTextEdit(this);
    // experimentProtocol_->setPlaceholderText("Enter experiment protocol details here...");
    experimentProtocol_->setMinimumHeight(40);
    QVBoxLayout *protocolLayout = new QVBoxLayout();
    protocolLayout->addWidget(protocolLabel);
    protocolLayout->addWidget(experimentProtocol_);

    // Save directory widget
    directoryLineEdit_ = new QLineEdit(this);
    directoryLineEdit_->setText(saveDirectory->getDirectory().c_str());
    connect(directoryLineEdit_,
            &QLineEdit::textChanged,
            this,
            [this, saveDirectory](const QString &text)
            { spdlog::debug("saveDirectory changed to {}", text.toStdString());
                saveDirectory->setDirectory(text.toStdString()); });
    QPushButton *browseButton = new QPushButton("Browse", this);

    QHBoxLayout *directoryLayout = new QHBoxLayout();
    directoryLayout->addWidget(new QLabel("Save Directory"));
    directoryLayout->addWidget(directoryLineEdit_);
    directoryLayout->addWidget(browseButton);

    connect(browseButton,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::browseDirectory);

    // Optional features checkboxes: tracking and muscle imaging
    QHBoxLayout *optionalFeaturesLayout = new QHBoxLayout();
    optionalFeaturesLayout->addWidget(new QLabel("Optional features"));
    // Check box to enable/disable tracking
    trackingEnabledCheckBox_ = new QCheckBox("Enable tracking", this);
    trackingEnabledCheckBox_->setChecked(true);
    connect(trackingEnabledCheckBox_,
            &QCheckBox::checkStateChanged,
            this,
            [this, trackingControlState](int state)
            {
                if (state == Qt::Checked)
                {
                    trackingControlState->trackingOn.store(true);
                }
                else
                {
                    trackingControlState->trackingOn.store(false);
                }
            });
    optionalFeaturesLayout->addWidget(trackingEnabledCheckBox_);

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
    imageDisplayTimer_->start(1000 / streamingBehaviorFPS_);
    connect(recordButton_,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::startRecording);
    connect(stopButton_,
            &QPushButton::clicked,
            this,
            &MainGUIWindow::stopRecording);

    // Add timer to keep checking for programmedRecordingStop
    QTimer *programmedStopCheckTimer = new QTimer(this);
    connect(programmedStopCheckTimer,
            &QTimer::timeout,
            this,
            [this, programmedRecordingStop]()
            {
                if (programmedRecordingStop->hasEndedFlagForGUI.load())
                {
                    spdlog::info("Protocol stop reached. Stopping recording.");
                    stopRecording();
                    programmedRecordingStop->numBehaviorFramesExpected = -1;
                    programmedRecordingStop->numMuscleFramesExpected = -1;
                    programmedRecordingStop
                        ->hasEndedFlagForGUI.store(false); // toggle off
                    QMessageBox::information(
                        this,
                        "Recording stopped",
                        "End of protocol reached. Recording stopped.");
                }
            });
    programmedStopCheckTimer->start(500); // Check every 0.5 second

    // Arrange layout
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addLayout(behaviorFPSLayout);
    layout->addLayout(syncRatioLayout);
    layout->addLayout(behaviorExposureTimeLayout);
    layout->addLayout(muscleExposureTimeLayout);
    layout->addLayout(protocolLayout);
    layout->addLayout(directoryLayout);
    layout->addLayout(optionalFeaturesLayout);
    layout->addWidget(behaviorImageDisplayLabel_);
    layout->addWidget(motionControlWidget_);
    layout->addLayout(recordStopButtonsLayout);
    setLayout(layout);

    // Initialy stream images only, don't save
    stopRecording();
}

void MainGUIWindow::startRecording()
{
    // Check if behavior camera has been initialized
    if (!behaviorRecordingState_->behaviorCamera ||
        !behaviorRecordingState_->behaviorCamera->isReady())
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

    // Toggle GUI buttons
    recordButton_->setEnabled(false);
    stopButton_->setEnabled(true);

    // Parse and set experiment protocol
    std::vector<ProtocolStep> protocolSteps;
    int numStepsParsed = parseProtocolString(
        experimentProtocol_->toPlainText().toStdString(), protocolSteps);
    spdlog::info("Parsed {} protocol steps", protocolSteps.size());
    if (numStepsParsed < 0)
    {
        std::string errorMessage = "Invalid experiment protocol string";
        spdlog::error(errorMessage);
        QMessageBox::critical(nullptr, "Error", errorMessage.c_str());
        return;
    }
    else if (numStepsParsed == 0)
    {
        spdlog::info("GUI starting recording without any protocol steps");
        programmedRecordingStop_->numBehaviorFramesExpected = -1;
        programmedRecordingStop_->numMuscleFramesExpected = -1;
    }
    else
    {
        spdlog::info("GUI starting recording with {} protocol steps",
                     protocolSteps.size());
        programmedRecordingStop_->numBehaviorFramesExpected =
            protocolSteps.back().frameCount;
        programmedRecordingStop_->numMuscleFramesExpected =
            protocolSteps.back().frameCount / syncRatioSpinBox_->value();
        spdlog::info(
            "Setting expected number of steps to {} (behavior) and {} (muscle)",
            programmedRecordingStop_->numBehaviorFramesExpected,
            programmedRecordingStop_->numMuscleFramesExpected);
    }

    // Initialize save directory
    saveDirectory_->initialize();

    // Save metadata: experiment protocol
    std::string protocolString = generateProtocolString(protocolSteps);
    std::filesystem::path metadataFilePath =
        saveDirectory_->getDirectory() / "metadata" / "experiment_protocol.txt";
    std::ofstream metadataFile(metadataFilePath);
    metadataFile << protocolString;
    metadataFile.close();
    spdlog::info("Saved experiment protocol to {}", metadataFilePath.string());

    // Save metadata: recording config
    std::filesystem::path recordingConfigFilePath =
        saveDirectory_->getDirectory() / "metadata" / "recording_config.yaml";
    recorderConfig_.saveToFile(recordingConfigFilePath);

    // Send triggering parameters to Arduino and start recording
    arduinoCommunication_->setBehaviorRecordingFPS(behaviorFPSSpinBox_->value());
    arduinoCommunication_->setSyncRatio(syncRatioSpinBox_->value());
    arduinoCommunication_->startRecording(protocolSteps);

    // Arduino will pause 100ms before starting triggering. This is to leave
    // some time to currently dangling, unprocessed time to pass through the
    // image saver thread. This way, when the image acquirer thread receives
    // any new frame, we know that they are part of the recording (ie. the
    // first frame that arrives should carry frame index 0). On the comptuer's
    // side, we will wait 80ms before setting isRecording to true. During these
    // 80ms, any frame that is received is still treated as streamed input for
    // visualization GUI. By contrast, any frame that arrives after 80ms is
    // considered part of the recording.
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    programState_->isRecording.store(true);
}

void MainGUIWindow::stopRecording()
{
    recordButton_->setEnabled(true);
    stopButton_->setEnabled(false);

    arduinoCommunication_->stopRecording();
    programState_->isRecording.store(false);
    arduinoCommunication_->setBehaviorRecordingFPS(streamingBehaviorFPS_);
    arduinoCommunication_->setSyncRatio(streamingSyncRatio_);
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
    std::string currentDirectory = saveDirectory_->getDirectory();
    QString dir = QFileDialog::getExistingDirectory(
        this,
        "Open Directory",
        QString::fromStdString(currentDirectory),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty())
    {
        directoryLineEdit_->setText(dir);

        saveDirectory_->setDirectory(dir.toStdString());
        spdlog::info("Directory changed to '{}'", dir.toStdString());
    }
    else
    {
        spdlog::error("Directory is an empty string; failed to open.");
    }
}

cv::Mat addCornerMarker(cv::Mat image,
                        int arenaSizeXmm,
                        int arenaSizeYmm,
                        MotionStagePosition stagePosition,
                        CalibrationParams &behaviorCamCalibrationParams)
{
    cv::Mat imageForDisplay = image.clone();
    assert(imageForDisplay.size() == image.size());

    std::vector<std::tuple<double, double>> cornerPositions = {
        {0, 0},
        {arenaSizeXmm, 0},
        {arenaSizeXmm, arenaSizeYmm},
        {0, arenaSizeYmm}};

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
    // if (!behaviorRecordingState_->latestBehaviorFrameHolder)
    // {
    //     spdlog::warn("Latest frame is empty. Cannot update image display.");
    //     return;
    // }
    cv::Mat latestFrame = behaviorRecordingState_
                              ->latestBehaviorFrameHolder
                              ->getLatestFrameData()
                              .image;
    if (latestFrame.empty())
    {
        return;
    }
    cv::Mat correctedFrame = reorientBehaviorImage(latestFrame);

    MotionStagePosition myStagePosition;
    {
        std::lock_guard<std::mutex> lock(
            trackingControlState_->latestMotionStagePositionMutex);
        myStagePosition = trackingControlState_->latestMotionStagePosition;
    }

    cv::Mat maskedImage = blackoutOutside(
        correctedFrame,
        myStagePosition,
        behaviorCamCalibrationParams_,
        recorderConfig_);

    int arenaSizeXmm = recorderConfig_.getParameter<int>("arena", "size_x_mm");
    int arenaSizeYmm = recorderConfig_.getParameter<int>("arena", "size_y_mm");
    cv::Mat imageForDisplay = addCornerMarker(maskedImage,
                                              arenaSizeXmm,
                                              arenaSizeYmm,
                                              myStagePosition,
                                              behaviorCamCalibrationParams_);

    QImage qImage = cvMatToQImage(imageForDisplay);
    QPixmap pixmap = QPixmap::fromImage(qImage)
                         .scaled(behaviorImageDisplayLabel_->size(),
                                 Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
    behaviorImageDisplayLabel_->setPixmap(pixmap);
}

int parseProtocolString(
    const std::string &protocolTextFieldString,
    std::vector<ProtocolStep> &protocolSteps)
/**
 * This for now is very repetative, but it's intended we can rewrite this
 * function based on nicer GUI widgets.
 * (Though for now it's just doing parseProtocolSequence, only to be later
 * formatted into the exact same strings)
 */
{
    int numStepsParsed = parseProtocolSequence(protocolTextFieldString,
                                               protocolSteps);
    if (numStepsParsed < 0)
    {
        std::string errorMessage = "Invalid experiment protocol";
        spdlog::error(errorMessage);
        QMessageBox::critical(nullptr, "Error", errorMessage.c_str());
        return -1;
    }
    return numStepsParsed;
}