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
    double minXAbsoluteMm,
    double maxXAbsoluteMm,
    double minYAbsoluteMm,
    double maxYAbsoluteMm,
    QWidget *parent)
    : QWidget(parent),
      trackingControlState_(trackingControlState)
{
    // Stage bounds are now derived externally (in runSpotlight_main) from
    // the arena dimensions and the fitted calibration model, instead of
    // being read from removed motion_control.x_min_mm/etc. config keys.
    minXAbsoluteMm_ = minXAbsoluteMm;
    maxXAbsoluteMm_ = maxXAbsoluteMm;
    minYAbsoluteMm_ = minYAbsoluteMm;
    maxYAbsoluteMm_ = maxYAbsoluteMm;

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
    std::shared_ptr<DualRecordingConfig> dualRecordingConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<TrackingControlState> trackingControlState,
    CalibrationParams &behaviorCamCalibrationParams,
    CalibrationParams &muscleCamCalibrationParams,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ArduinoCommunication> arduinoCommunication,
    std::shared_ptr<ProgramState> programState,
    std::shared_ptr<ProgrammedStop> programmedRecordingStop,
    double stageMinXMm,
    double stageMaxXMm,
    double stageMinYMm,
    double stageMaxYMm,
    QWidget *parent)
    : QWidget(parent),
      recorderConfig_(recorderConfig),
      behaviorRecordingState_(behaviorRecordingState),
      muscleRecordingState_(muscleRecordingState),
      trackingControlState_(trackingControlState),
      behaviorCamCalibrationParams_(behaviorCamCalibrationParams),
      muscleCamCalibrationParams_(muscleCamCalibrationParams),
      saveDirectory_(saveDirectory),
      arduinoCommunication_(arduinoCommunication),
      programState_(programState),
      programmedRecordingStop_(programmedRecordingStop),
      dualRecordingConfigForSaving_(dualRecordingConfig),
      stageMinXMm_(stageMinXMm),
      stageMaxXMm_(stageMaxXMm),
      stageMinYMm_(stageMinYMm),
      stageMaxYMm_(stageMaxYMm)
{
    streamingBehaviorFPS_ = recorderConfig.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");

    // Load parameters for displaying 16-bit muscle image
    muscleImage16To8BitScale_ = recorderConfig.getParameter<int>(
        "muscle_camera", "conversion_16to8bit_scale_camera_alignment");
    muscleImage16To8BitOffset_ = recorderConfig.getParameter<int>(
        "muscle_camera", "conversion_16to8bit_offset_camera_alignment");

    // Load rolling shutter parameter
    double rollingShutterLineTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    int muscleCamReadoutTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "sensor_readout_time_us");

    // Load streaming sync ratio
    streamingSyncRatio_ = recorderConfig.getParameter<int>(
        "muscle_camera", "streaming_sync_ratio");
    dualRecordingConfigForStreaming_ =
        std::make_unique<DualRecordingConfig>(
            streamingBehaviorFPS_,
            streamingSyncRatio_,
            dualRecordingConfig->getMuscleLightOnTimeUs());
    size_t retryCount = 0;
    while (muscleRecordingState_->muscleCamera == nullptr)
    {
        // Wait for the muscle camera to be initialized
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retryCount++;
        if (retryCount % 10 == 0)
        {
            spdlog::warn("Muscle camera is not initialized.");
        }
    }
    dualRecordingConfigForStreaming_->computeParameters(
        muscleRecordingState_->muscleCamera->getNumLinesScanned(),
        rollingShutterLineTimeUs,
        muscleCamReadoutTimeUs);

    // Behavior FPS widget
    behaviorFPSSpinBox_ = new QSpinBox(this);
    behaviorFPSSpinBox_->setRange(1, 1000);
    int behaviorCameraDefaultRecordingFrameRate =
        recorderConfig.getParameter<int>("behavior_camera",
                                         "default_recording_fps");
    behaviorFPSSpinBox_->setValue(behaviorCameraDefaultRecordingFrameRate);
    if (dualRecordingConfig->isRecordingBoth())
    {
        // If dual recording is enabled, set the value from the config
        // and disable the spin box to prevent changes.
        spdlog::debug("Setting behavior FPS to {}",
                      dualRecordingConfig->getBehaviorCameraFPS());
        behaviorFPSSpinBox_->setValue(
            dualRecordingConfig->getBehaviorCameraFPS());
        behaviorFPSSpinBox_->setEnabled(false);
    }
    // Don't connect to ArduinoCommunication! This value is only used during
    // recording. When streaming, the sync ratio is always 1 and this field is
    // ignored.
    QHBoxLayout *behaviorFPSLayout = new QHBoxLayout();
    behaviorFPSLayout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behaviorFPSLayout->addWidget(behaviorFPSSpinBox_);

    // Behavior-muscle synchronization ratio
    syncRatioSpinBox_ = new QSpinBox(this);
    syncRatioSpinBox_->setRange(1, INT_MAX);
    int syncRatio = recorderConfig.getParameter<int>(
        "muscle_camera", "default_recording_sync_ratio");
    syncRatioSpinBox_->setValue(syncRatio);
    if (dualRecordingConfig->isRecordingBoth())
    {
        syncRatioSpinBox_->setValue(dualRecordingConfig->getSyncRatio());
        syncRatioSpinBox_->setEnabled(false);
    }
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
    muscleLightOnTimeSpinBox_ = new QDoubleSpinBox(this);
    muscleLightOnTimeSpinBox_->setRange(0.001, 1000.0);
    int muscleCameraDefaultLightOnTimeUs = recorderConfig.getParameter<int>(
        "muscle_camera", "default_light_on_time_us");
    muscleLightOnTimeSpinBox_->setValue(
        muscleCameraDefaultLightOnTimeUs / 1000.0);
    if (dualRecordingConfig->isRecordingBoth())
    {
        muscleLightOnTimeSpinBox_->setValue(
            dualRecordingConfig->getMuscleLightOnTimeUs() / 1000.0);
        muscleLightOnTimeSpinBox_->setEnabled(false);
    }
    connect(muscleLightOnTimeSpinBox_,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this, arduinoCommunication](double value)
            { arduinoCommunication->setMuscleLightOnTime(value * 1000); });
    QHBoxLayout *muscleLightOnTimeLayout = new QHBoxLayout();
    muscleLightOnTimeLayout->addWidget(
        new QLabel("Muscle exposure (light-on) time (ms)"));
    muscleLightOnTimeLayout->addWidget(muscleLightOnTimeSpinBox_);

    // Experiment protocol widget
    QLabel *protocolLabel = new QLabel("Experiment protocol", this);
    experimentProtocol_ = new QTextEdit(this);
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

    // Check box to enable/disable muscle imaging
    muscleImagingCheckBox_ = new QCheckBox("Enable muscle imaging", this);
    muscleImagingCheckBox_->setChecked(false);
    connect(muscleImagingCheckBox_,
            &QCheckBox::checkStateChanged,
            this,
            [this, arduinoCommunication](int state)
            {
                if (state == Qt::Checked)
                {
                    spdlog::info("Enabling muscle imaging, "
                                 "setting sync ratio to {}",
                                 streamingSyncRatio_);
                    muscleImagingEnabled_ = true;
                    arduinoCommunication->setSyncRatio(streamingSyncRatio_);
                    arduinoCommunication->setMuscleCamTriggerDelay(
                        dualRecordingConfigForStreaming_
                            ->getMuscleCamTriggerDelayUs());
                }
                else
                {
                    spdlog::info("Disabling muscle imaging, "
                                 "setting sync ratio to INT_MAX");
                    muscleImagingEnabled_ = false;
                    arduinoCommunication->setSyncRatio(INT_MAX);
                }
            });
    if (!dualRecordingConfig->isRecordingBoth())
    {
        muscleImagingCheckBox_->setEnabled(false);
    }
    optionalFeaturesLayout->addWidget(muscleImagingCheckBox_);

    // Live image displays
    QHBoxLayout *liveImageDisplayLayout = new QHBoxLayout();

    // Behavior camera preview
    behaviorImageDisplayLabel_ = new QLabel(this);
    int behaviorCameraPreviewWidth = recorderConfig.getParameter<int>(
        "gui", "behavior_camera_preview_width");
    int behaviorCameraPreviewHeight = recorderConfig.getParameter<int>(
        "gui", "behavior_camera_preview_height");
    behaviorImageDisplayLabel_->setFixedSize(behaviorCameraPreviewWidth,
                                             behaviorCameraPreviewHeight);
    liveImageDisplayLayout->addWidget(behaviorImageDisplayLabel_);
    // Add timer to for behavior display updates
    imageDisplayTimer_ = new QTimer(this);
    connect(imageDisplayTimer_,
            &QTimer::timeout,
            this,
            &MainGUIWindow::updateBehaviorImageDisplay);
    imageDisplayTimer_->start(1000 / streamingBehaviorFPS_);

    // Muscle camera preview
    muscleImageDisplayLabel_ = new QLabel(this);
    int muscleCameraPreviewWidth = recorderConfig.getParameter<int>(
        "gui", "muscle_camera_preview_width");
    int muscleCameraPreviewHeight = recorderConfig.getParameter<int>(
        "gui", "muscle_camera_preview_height");
    muscleImageDisplayLabel_->setFixedSize(muscleCameraPreviewWidth,
                                           muscleCameraPreviewHeight);
    liveImageDisplayLayout->addWidget(muscleImageDisplayLabel_);
    // Add timer for muscle display updates
    QTimer *muscleImageDisplayTimer = new QTimer(this);
    connect(muscleImageDisplayTimer,
            &QTimer::timeout,
            this,
            &MainGUIWindow::updateMuscleImageDisplay);
    float muscleStreamingFPS =
        static_cast<float>(streamingBehaviorFPS_) / streamingSyncRatio_;
    spdlog::info("Muscle streaming FPS: {}", muscleStreamingFPS);
    muscleImageDisplayTimer->start(1000 / muscleStreamingFPS);

    // Motion stage state display
    motionControlWidget_ = new MotionControlWidget(recorderConfig,
                                                   trackingControlState,
                                                   stageMinXMm_,
                                                   stageMaxXMm_,
                                                   stageMinYMm_,
                                                   stageMaxYMm_,
                                                   this);

    // Record and stop buttons
    recordButton_ = new QPushButton("Record", this);
    stopButton_ = new QPushButton("Stop", this);
    stopButton_->setEnabled(false); // initially disabled
    QHBoxLayout *recordStopButtonsLayout = new QHBoxLayout();
    recordStopButtonsLayout->addWidget(recordButton_);
    recordStopButtonsLayout->addWidget(stopButton_);
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
    layout->addLayout(muscleLightOnTimeLayout);
    layout->addLayout(protocolLayout);
    layout->addLayout(directoryLayout);
    layout->addLayout(optionalFeaturesLayout);
    layout->addLayout(liveImageDisplayLayout);
    layout->addWidget(motionControlWidget_);
    layout->addLayout(recordStopButtonsLayout);
    setLayout(layout);

    // Initially set excitation light on time to correct value
    while (!muscleRecordingState->muscleCamera)
    {
        spdlog::info("Waiting for muscle camera to be ready...");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    arduinoCommunication->setMuscleLightOnTime(
        dualRecordingConfigForSaving_->getMuscleLightOnTimeUs());

    // Muscle imaging disabled by default
    arduinoCommunication_->setSyncRatio(INT_MAX);

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
    muscleImagingCheckBox_->setEnabled(false);

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

    // Save metadata: experiment parameters
    std::filesystem::path experimentParametersFilePath =
        saveDirectory_->getDirectory() / "metadata/experiment_parameters.yaml";
    writeExperimentParameters(
        experimentParametersFilePath,
        behaviorFPSSpinBox_->value(),
        muscleImagingCheckBox_->isChecked(),
        syncRatioSpinBox_->value(),
        behaviorExposureTimeSpinBox_->value(),
        muscleLightOnTimeSpinBox_->value(),
        experimentProtocol_->toPlainText().toStdString());
    spdlog::info(
        "Saved experiment parameters to '{}'",
        experimentParametersFilePath.string());

    // Save metadata: recording config
    std::filesystem::path recordingConfigFilePath =
        saveDirectory_->getDirectory() / "metadata/recorder_config.yaml";
    recorderConfig_.saveToFile(recordingConfigFilePath);
    spdlog::info(
        "Saved recorder config to '{}'", recordingConfigFilePath.string());

    // Save metadata: calibration parameters
    std::filesystem::path behaviorCalibrationFilePath =
        saveDirectory_->getDirectory() /
        "metadata/calibration_parameters_behavior.yaml";
    behaviorCamCalibrationParams_.saveToFile(behaviorCalibrationFilePath);
    spdlog::info("Saved behavior calibration parameters to '{}'",
                 behaviorCalibrationFilePath.string());
    if (muscleCamCalibrationParams_.isDefined)
    {
        std::filesystem::path muscleCalibrationFilePath =
            saveDirectory_->getDirectory() /
            "metadata/calibration_parameters_muscle.yaml";
        muscleCamCalibrationParams_.saveToFile(muscleCalibrationFilePath);
        spdlog::info("Saved muscle calibration parameters to '{}'",
                     muscleCalibrationFilePath.string());
    }
    else
    {
        spdlog::warn(
            "Muscle camera calibration parameters not defined. Not saving.");
    }

    // Save timing metadata
    if (dualRecordingConfigForSaving_->isRecordingBoth())
    {
        std::filesystem::path behaviorCalibrationFilePath =
            saveDirectory_->getDirectory() /
            "metadata/dual_recording_timing.yaml";
        dualRecordingConfigForSaving_->saveToFile(behaviorCalibrationFilePath);
        spdlog::info("Saved dual recording timing parameters to '{}'",
                     behaviorCalibrationFilePath.string());
    }

    // Send triggering parameters to Arduino and start recording
    arduinoCommunication_->setBehaviorRecordingFPS(
        behaviorFPSSpinBox_->value());
    arduinoCommunication_->setSyncRatio(
        dualRecordingConfigForSaving_->getSyncRatio());
    arduinoCommunication_->setMuscleCamTriggerDelay(
        dualRecordingConfigForSaving_->getMuscleCamTriggerDelayUs());
    arduinoCommunication_->startRecording(protocolSteps);

    // Arduino will pause 100ms before starting triggering. This is to leave
    // some time to currently dangling, unprocessed time to pass through the
    // image saver thread. This way, when the image acquirer thread receives
    // any new frame, we know that they are part of the recording (ie. the
    // first frame that arrives shnum_image_saving_thredsntrast, any frame that
    // arrives after 80ms is considered part of the recording.
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    programState_->isRecording.store(true);
}

void MainGUIWindow::stopRecording()
{
    recordButton_->setEnabled(true);
    stopButton_->setEnabled(false);
    muscleImagingCheckBox_->setEnabled(true);

    arduinoCommunication_->stopRecording();
    programState_->isRecording.store(false);
    arduinoCommunication_->setBehaviorRecordingFPS(streamingBehaviorFPS_);
    if (muscleImagingEnabled_)
    {
        arduinoCommunication_->setSyncRatio(streamingSyncRatio_);
        arduinoCommunication_->setMuscleCamTriggerDelay(
            dualRecordingConfigForStreaming_->getMuscleCamTriggerDelayUs());
    }
    else
    {
        arduinoCommunication_->setSyncRatio(INT_MAX);
    }
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
        pixelPoints.emplace_back(pixelCol, pixelRow);
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

void MainGUIWindow::updateBehaviorImageDisplay()
{
    cv::Mat latestFrame = behaviorRecordingState_
                              ->latestFrameHolder
                              ->getLatestFrameData()
                              .image;
    if (latestFrame.empty())
    {
        return;
    }
    cv::Mat correctedFrame;
    reorientBehaviorImage(latestFrame, correctedFrame);

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

void MainGUIWindow::updateMuscleImageDisplay()
{
    if (!muscleImagingEnabled_)
    {
        muscleImageDisplayLabel_->clear();
        return;
    }

    cv::Mat latestFrame = muscleRecordingState_
                              ->latestFrameHolder
                              ->getLatestFrameData()
                              .image;
    if (latestFrame.empty())
    {
        return;
    }
    cv::Mat processedFrame;
    convert16BitTo8Bit(latestFrame,
                       processedFrame,
                       muscleImage16To8BitScale_,
                       muscleImage16To8BitOffset_);
    reorientMuscleImage(processedFrame, processedFrame);
    QImage qImage = cvMatToQImage(processedFrame);
    QPixmap pixmap = QPixmap::fromImage(qImage)
                         .scaled(muscleImageDisplayLabel_->size(),
                                 Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
    muscleImageDisplayLabel_->setPixmap(pixmap);
}

DualRecordingConfigWindow::DualRecordingConfigWindow(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<DualRecordingConfig> dualRecordingConfig,
    const MuscleCameraROI &muscleCameraROI,
    QWidget *parent)
    : QDialog(parent),
      dualRecordingConfigForSaving_(dualRecordingConfig),
      recorderConfig_(recorderConfig),
      muscleCameraROI_(muscleCameraROI)
{
    setWindowTitle("Dual recording configuration");
    mainLayout_ = new QVBoxLayout(this);
    resize(desiredWidth_, desiredHeight_);

    // Block for recording behavior only
    QLabel *labelTitle = new QLabel(
        "<b>Recording both behavior and muscle</b>");
    QLabel *labelNoMuscle = new QLabel(
        "<i>If you wish to record behavior only</i>, click the button below.");
    labelNoMuscle->setWordWrap(true);
    withoutMuscleButton_ = new QPushButton("Record behavior only", this);
    connect(withoutMuscleButton_,
            &QPushButton::clicked,
            this,
            &DualRecordingConfigWindow::onButtonClicked);
    mainLayout_->addWidget(labelTitle);
    mainLayout_->addWidget(labelNoMuscle);
    mainLayout_->addWidget(withoutMuscleButton_);
    mainLayout_->addSpacing(10);

    // Block for dual recording
    QLabel *labelWithMuscle = new QLabel(
        "<i>If you wish to record both behavior and muscle</i>, complete the "
        "following settings and click the button below. Once the program "
        "starts, you will not be able to change these settings. If you want "
        "to change them, you will have to restart the program.");
    labelWithMuscle->setWordWrap(true);
    mainLayout_->addWidget(labelWithMuscle);

    // Behavior FPS
    QHBoxLayout *behaviorCameraFPSLayout = new QHBoxLayout();
    QLabel *behaviorCameraFPSLabel = new QLabel(
        "Behavior camera FPS (Hz)", this);
    behaviorCameraFPSSpinBox_ = new QSpinBox(this);
    behaviorCameraFPSSpinBox_->setRange(1, 1000);
    behaviorCameraFPSSpinBox_->setValue(
        recorderConfig.getParameter<int>(
            "behavior_camera", "default_recording_fps"));
    behaviorCameraFPSLayout->addWidget(behaviorCameraFPSLabel);
    behaviorCameraFPSLayout->addWidget(behaviorCameraFPSSpinBox_);
    mainLayout_->addLayout(behaviorCameraFPSLayout);

    // Behavior-muscle sync ratio
    QHBoxLayout *syncRatioLayout = new QHBoxLayout();
    QLabel *syncRatioLabel = new QLabel(
        "Sync ratio (behavior FPS : muscle FPS)", this);
    syncRatioSpinBox_ = new QSpinBox(this);
    syncRatioSpinBox_->setRange(1, 100);
    syncRatioSpinBox_->setValue(
        recorderConfig.getParameter<int>("muscle_camera",
                                         "default_recording_sync_ratio"));
    syncRatioLayout->addWidget(syncRatioLabel);
    syncRatioLayout->addWidget(syncRatioSpinBox_);
    mainLayout_->addLayout(syncRatioLayout);

    // Muscle exposure time
    QHBoxLayout *muscleLightOnTimeLayout = new QHBoxLayout();
    QLabel *muscleLightOnTimeLabel = new QLabel(
        "Muscle exposure (light-on) time (ms)", this);
    muscleLightOnTimeSpinBox_ = new QDoubleSpinBox(this);
    muscleLightOnTimeSpinBox_->setRange(0.001, 10000.0);
    muscleLightOnTimeSpinBox_->setValue(
        recorderConfig.getParameter<int>("muscle_camera",
                                         "default_light_on_time_us") /
        1000.0);
    muscleLightOnTimeLayout->addWidget(muscleLightOnTimeLabel);
    muscleLightOnTimeLayout->addWidget(muscleLightOnTimeSpinBox_);
    mainLayout_->addLayout(muscleLightOnTimeLayout);

    withMuscleButton_ = new QPushButton("Record behavior and muscle", this);
    connect(withMuscleButton_,
            &QPushButton::clicked,
            this,
            &DualRecordingConfigWindow::onButtonClicked);
    mainLayout_->addWidget(withMuscleButton_);
}

DualRecordingConfigWindow::~DualRecordingConfigWindow()
{
    // Nothing to clean up here, as we are using Qt's parent-child
}

void DualRecordingConfigWindow::onButtonClicked()
{
    dualRecordingConfigForSaving_->setRecordBoth(sender() == withMuscleButton_);
    dualRecordingConfigForSaving_->setBehaviorCameraFPS(
        behaviorCameraFPSSpinBox_->value());
    dualRecordingConfigForSaving_->setSyncRatio(syncRatioSpinBox_->value());
    dualRecordingConfigForSaving_->setMuscleLightOnTimeUs(
        static_cast<int>(muscleLightOnTimeSpinBox_->value() * 1000));

    int muscleImageHeight = muscleCameraROI_.imageHeight;
    double muscleCameraLineScanTimeUs = recorderConfig_.getParameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    int muscleCameraReadoutTimeUs = recorderConfig_.getParameter<double>(
        "muscle_camera", "sensor_readout_time_us");

    bool isValid =
        dualRecordingConfigForSaving_->computeParameters(muscleImageHeight,
                                                muscleCameraLineScanTimeUs,
                                                muscleCameraReadoutTimeUs);

    if (!isValid)
    {
        QMessageBox::critical(
            this,
            "Error",
            "Invalid configuration for recording both behavior and muscle. "
            "Please check the parameters and try again. "
            "In particular, check if the muscle recording interval "
            "(i.e. 1 / muscleFPS) is greater than the minimum. See "
            "https://github.com/NeLy-EPFL/spotlight-control/issues/79.");
    }
    else
    {
        spdlog::info("Behavior-muscle corecording config set: "
                     "recordBoth={}, behaviorCameraFPS={}, syncRatio={}, "
                     "muscleLightOnTimeUs={}",
                     dualRecordingConfigForSaving_->isRecordingBoth(),
                     dualRecordingConfigForSaving_->getBehaviorCameraFPS(),
                     dualRecordingConfigForSaving_->getSyncRatio(),
                     dualRecordingConfigForSaving_->getMuscleLightOnTimeUs());
        accept(); // close the dialog
    }
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
