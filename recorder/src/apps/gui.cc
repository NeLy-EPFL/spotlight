#include "recorder/apps/gui.h"

#include <iomanip>
#include <sstream>

namespace {
QImage cvMatToQImage(const cv::Mat &mat) {
    if (mat.empty()) {
        return QImage();
    }
    if (mat.channels() == 1) {
        return QImage(
            mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
    }
    // 3-channel BGR -> RGB for Qt
    cv::Mat rgb;
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888)
        .copy();
}
} // namespace

std::string incrementDirectoryName(const std::string &path) {
    size_t end = path.size();
    while (end > 0 && path[end - 1] == '/') {
        --end;
    }
    size_t start = end;
    while (start > 0 &&
           std::isdigit(static_cast<unsigned char>(path[start - 1]))) {
        --start;
    }

    if (start == end)
        return path.substr(0, end) + "_001/";

    std::string numStr = path.substr(start, end - start);
    int num = std::stoi(numStr) + 1;
    int width = static_cast<int>(numStr.size());

    std::ostringstream oss;
    oss << path.substr(0, start) << std::setfill('0') << std::setw(width) << num
        << "/";
    return oss.str();
}

MotionControlWidget::MotionControlWidget(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<TrackingControlState> trackingControlState,
    double minXAbsoluteMm,
    double maxXAbsoluteMm,
    double minYAbsoluteMm,
    double maxYAbsoluteMm,
    QWidget *parent)
    : QWidget(parent), trackingControlState_(trackingControlState) {
    // Stage bounds are now derived externally (in runSpotlight_main) from
    // the arena dimensions and the fitted calibration model, instead of
    // being read from removed motion_control.x_min_mm/etc. config keys.
    minXAbsoluteMm_ = minXAbsoluteMm;
    maxXAbsoluteMm_ = maxXAbsoluteMm;
    minYAbsoluteMm_ = minYAbsoluteMm;
    maxYAbsoluteMm_ = maxYAbsoluteMm;

    int guiMotionStagePreviewUpdateFreq = recorderConfig.getParameter<int>(
        "gui", "motion_stage_preview_update_frequency_hz");
    int guiMotionStagePreviewHeight =
        recorderConfig.getParameter<int>("gui", "motion_stage_preview_height");

    connect(
        &timer_,
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

MotionControlWidget::~MotionControlWidget() {
    timer_.stop();
}

void MotionControlWidget::paintEvent(QPaintEvent *event) {
    if (!trackingControlState_->motionControlHandlerReady.load()) {
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
    const float physicalX = currStagePosition.xPosMm;
    const float physicalY = currStagePosition.yPosMm;
    int pixelX = mapToPixelX(physicalX);
    int pixelY = mapToPixelY(physicalY);

    // Draw the red dot
    painter.setPen(Qt::red);
    painter.setBrush(Qt::red);
    int dotDiameter = 10;
    painter.drawEllipse(
        pixelX - dotDiameter / 2,
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
        textPositionX,
        textPositionY,
        QString("(%1, %2) mm")
            .arg(physicalX, minFieldWidth, 'f', precision)
            .arg(physicalY, minFieldWidth, 'f', precision));
}

void MotionControlWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        float stageX = mapToStageX(event->position().x());
        float stageY = mapToStageY(event->position().y());
        spdlog::debug("Clicked at ({}, {})", stageX, stageY);
        trackingControlState_->overridingPosX.store(stageX);
        trackingControlState_->overridingPosY.store(stageY);
        trackingControlState_->shouldOverrideTracking.store(true);
    }
}

int MotionControlWidget::mapToPixelX(float x) const {
    return (x - minXAbsoluteMm_) / (maxXAbsoluteMm_ - minXAbsoluteMm_) *
               width() +
           0.5;
}

int MotionControlWidget::mapToPixelY(float y) const {
    return (y - minYAbsoluteMm_) / (maxYAbsoluteMm_ - minYAbsoluteMm_) *
               height() +
           0.5;
}

float MotionControlWidget::mapToStageX(int x) const {
    return x / static_cast<float>(width()) *
               (maxXAbsoluteMm_ - minXAbsoluteMm_) +
           minXAbsoluteMm_;
}

float MotionControlWidget::mapToStageY(int y) const {
    return y / static_cast<float>(height()) *
               (maxYAbsoluteMm_ - minYAbsoluteMm_) +
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
    ActiveAreaMask &activeAreaMask,
    double stageMinXMm,
    double stageMaxXMm,
    double stageMinYMm,
    double stageMaxYMm,
    QWidget *parent)
    : QWidget(parent), recorderConfig_(recorderConfig),
      behaviorRecordingState_(behaviorRecordingState),
      muscleRecordingState_(muscleRecordingState),
      trackingControlState_(trackingControlState),
      behaviorCamCalibrationParams_(behaviorCamCalibrationParams),
      muscleCamCalibrationParams_(muscleCamCalibrationParams),
      saveDirectory_(saveDirectory),
      arduinoCommunication_(arduinoCommunication), programState_(programState),
      programmedRecordingStop_(programmedRecordingStop),
      dualRecordingConfigForSaving_(dualRecordingConfig),
      activeAreaMask_(activeAreaMask), stageMinXMm_(stageMinXMm),
      stageMaxXMm_(stageMaxXMm), stageMinYMm_(stageMinYMm),
      stageMaxYMm_(stageMaxYMm) {
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
    dualRecordingConfigForStreaming_ = std::make_unique<DualRecordingConfig>(
        streamingBehaviorFPS_,
        streamingSyncRatio_,
        dualRecordingConfig->getMuscleLightOnTimeUs());
    size_t retryCount = 0;
    while (muscleRecordingState_->muscleCamera == nullptr) {
        // Wait for the muscle camera to be initialized
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retryCount++;
        if (retryCount % 10 == 0) {
            spdlog::warn("Muscle camera is not initialized.");
        }
    }
    dualRecordingConfigForStreaming_->computeParameters(
        muscleRecordingState_->muscleCamera->getNumLinesScanned(),
        rollingShutterLineTimeUs,
        muscleCamReadoutTimeUs);

    // Cache the PCO sensor timing sent in every STREAM / START_RECORDING. The
    // controller derives the muscle trigger delay from these (the rolling time
    // is the time to scan all lines of the muscle ROI).
    pcoCamRollingTimeUs_ = static_cast<unsigned int>(
        muscleRecordingState_->muscleCamera->getNumLinesScanned() *
        rollingShutterLineTimeUs);
    pcoCamReadoutTimeUs_ = static_cast<unsigned int>(muscleCamReadoutTimeUs);

    // Behavior FPS widget
    behaviorFPSSpinBox_ = new QSpinBox(this);
    behaviorFPSSpinBox_->setRange(1, 1000);
    int behaviorCameraDefaultRecordingFrameRate =
        recorderConfig.getParameter<int>(
            "behavior_camera", "default_recording_fps");
    behaviorFPSSpinBox_->setValue(behaviorCameraDefaultRecordingFrameRate);
    if (dualRecordingConfig->isRecordingBoth()) {
        // If dual recording is enabled, set the value from the config
        // and disable the spin box to prevent changes.
        spdlog::debug(
            "Setting behavior FPS to {}",
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
    if (dualRecordingConfig->isRecordingBoth()) {
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
    connect(
        behaviorExposureTimeSpinBox_,
        QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this,
        [this](double) {
            // Only adjust live (streaming) params; sending a STREAM mid-recording
            // would revert the controller and abort the recording.
            if (!programState_->isRecording.load()) {
                arduinoCommunication_->stream(buildStreamingParams());
            }
        });
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
    if (dualRecordingConfig->isRecordingBoth()) {
        muscleLightOnTimeSpinBox_->setValue(
            dualRecordingConfig->getMuscleLightOnTimeUs() / 1000.0);
        muscleLightOnTimeSpinBox_->setEnabled(false);
    }
    connect(
        muscleLightOnTimeSpinBox_,
        QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this,
        [this](double) {
            if (!programState_->isRecording.load()) {
                arduinoCommunication_->stream(buildStreamingParams());
            }
        });
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
    connect(
        directoryLineEdit_,
        &QLineEdit::textChanged,
        this,
        [this, saveDirectory](const QString &text) {
            spdlog::debug("saveDirectory changed to {}", text.toStdString());
            saveDirectory->setDirectory(text.toStdString());
        });
    QPushButton *browseButton = new QPushButton("Browse", this);
    QPushButton *incrementButton = new QPushButton("Increment", this);

    QHBoxLayout *directoryLayout = new QHBoxLayout();
    directoryLayout->addWidget(new QLabel("Save Directory"));
    directoryLayout->addWidget(directoryLineEdit_);
    directoryLayout->addWidget(browseButton);
    directoryLayout->addWidget(incrementButton);

    connect(
        browseButton,
        &QPushButton::clicked,
        this,
        &MainGUIWindow::browseDirectory);
    connect(
        incrementButton,
        &QPushButton::clicked,
        this,
        &MainGUIWindow::incrementDirectory);

    // Optional features checkboxes: tracking and muscle imaging
    QHBoxLayout *optionalFeaturesLayout = new QHBoxLayout();
    optionalFeaturesLayout->addWidget(new QLabel("Optional features"));

    // Check box to enable/disable tracking
    trackingEnabledCheckBox_ = new QCheckBox("Enable tracking", this);
    trackingEnabledCheckBox_->setChecked(true);
    connect(
        trackingEnabledCheckBox_,
        &QCheckBox::checkStateChanged,
        this,
        [this, trackingControlState](int state) {
            if (state == Qt::Checked) {
                trackingControlState->trackingOn.store(true);
            } else {
                trackingControlState->trackingOn.store(false);
            }
        });
    optionalFeaturesLayout->addWidget(trackingEnabledCheckBox_);

    // Check box to enable/disable muscle imaging
    muscleImagingCheckBox_ = new QCheckBox("Enable muscle imaging", this);
    muscleImagingCheckBox_->setChecked(false);
    connect(
        muscleImagingCheckBox_,
        &QCheckBox::checkStateChanged,
        this,
        [this](int state) {
            // Muscle imaging on/off is expressed by enableMuscle: when off the
            // controller free-runs the behavior camera with the blue excitation
            // LED disabled (buildStreamingParams() reads muscleImagingEnabled_).
            if (state == Qt::Checked) {
                spdlog::info("Enabling muscle imaging");
                muscleImagingEnabled_ = true;
            } else {
                spdlog::info("Disabling muscle imaging");
                muscleImagingEnabled_ = false;
            }
            if (!programState_->isRecording.load()) {
                arduinoCommunication_->stream(buildStreamingParams());
            }
        });
    if (!dualRecordingConfig->isRecordingBoth()) {
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
    behaviorImageDisplayLabel_->setFixedSize(
        behaviorCameraPreviewWidth, behaviorCameraPreviewHeight);
    liveImageDisplayLayout->addWidget(behaviorImageDisplayLabel_);
    // Add timer to for behavior display updates
    imageDisplayTimer_ = new QTimer(this);
    connect(
        imageDisplayTimer_,
        &QTimer::timeout,
        this,
        &MainGUIWindow::updateBehaviorImageDisplay);
    imageDisplayTimer_->start(1000 / streamingBehaviorFPS_);

    // Muscle camera preview
    muscleImageDisplayLabel_ = new QLabel(this);
    int muscleCameraPreviewWidth =
        recorderConfig.getParameter<int>("gui", "muscle_camera_preview_width");
    int muscleCameraPreviewHeight =
        recorderConfig.getParameter<int>("gui", "muscle_camera_preview_height");
    muscleImageDisplayLabel_->setFixedSize(
        muscleCameraPreviewWidth, muscleCameraPreviewHeight);
    liveImageDisplayLayout->addWidget(muscleImageDisplayLabel_);
    // Add timer for muscle display updates
    QTimer *muscleImageDisplayTimer = new QTimer(this);
    connect(
        muscleImageDisplayTimer,
        &QTimer::timeout,
        this,
        &MainGUIWindow::updateMuscleImageDisplay);
    float muscleStreamingFPS =
        static_cast<float>(streamingBehaviorFPS_) / streamingSyncRatio_;
    spdlog::info("Muscle streaming FPS: {}", muscleStreamingFPS);
    muscleImageDisplayTimer->start(1000 / muscleStreamingFPS);

    // Motion stage state display
    motionControlWidget_ = new MotionControlWidget(
        recorderConfig,
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
    connect(
        recordButton_,
        &QPushButton::clicked,
        this,
        &MainGUIWindow::startRecording);
    connect(
        stopButton_,
        &QPushButton::clicked,
        this,
        &MainGUIWindow::stopRecording);

    // Add timer to keep checking for programmedRecordingStop
    QTimer *programmedStopCheckTimer = new QTimer(this);
    connect(
        programmedStopCheckTimer,
        &QTimer::timeout,
        this,
        [this, programmedRecordingStop]() {
            if (programmedRecordingStop->hasEndedFlagForGUI.load()) {
                spdlog::info("Protocol stop reached. Stopping recording.");
                endRecording(/*reachedProgrammedEnd=*/true);
                programmedRecordingStop->numBehaviorFramesExpected = -1;
                programmedRecordingStop->numMuscleFramesExpected = -1;
                programmedRecordingStop->hasEndedFlagForGUI.store(
                    false); // toggle off
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

    // Wait for the muscle camera before the first STREAM (its rolling/readout
    // timing feeds the trigger params).
    while (!muscleRecordingState->muscleCamera) {
        spdlog::info("Waiting for muscle camera to be ready...");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Start in streaming mode: live preview only, not saving. Muscle imaging is
    // off by default (enableMuscle = false: behavior camera free-runs, blue
    // excitation LED off). The controller is configured with a single STREAM
    // command.
    recordButton_->setEnabled(true);
    stopButton_->setEnabled(false);
    muscleImagingCheckBox_->setEnabled(true);
    programState_->isRecording.store(false);
    arduinoCommunication->stream(buildStreamingParams());
}

void MainGUIWindow::startRecording() {
    // Check if behavior camera has been initialized
    if (!behaviorRecordingState_->behaviorCamera ||
        !behaviorRecordingState_->behaviorCamera->isReady()) {
        spdlog::error("Behavior camera not ready. Cannot start recording.");
        // Make a pop-up error window
        QMessageBox::critical(
            this,
            "Error",
            "Behavior camera not ready yet. Please wait 10 seconds. If the "
            "error persists, something has gone wrong. Check logs for info.");
        return;
    }

    // Warn if the save directory already exists and is non-empty
    std::filesystem::path saveDir = saveDirectory_->getDirectory();
    while (std::filesystem::is_directory(saveDir) &&
           !std::filesystem::is_empty(saveDir)) {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Directory not empty");
        msgBox.setText(
            QString(
                "The save directory already exists and is non-empty:\n%1\n\n"
                "Overwrite its contents?")
                .arg(QString::fromStdString(saveDir.string())));
        msgBox.setIcon(QMessageBox::Warning);
        QPushButton *autoIncrementButton =
            msgBox.addButton("Auto increment", QMessageBox::ActionRole);
        QPushButton *overwriteButton =
            msgBox.addButton("Overwrite", QMessageBox::ActionRole);
        QPushButton *cancelButton =
            msgBox.addButton("Cancel", QMessageBox::ActionRole);
        msgBox.setEscapeButton(cancelButton);
        msgBox.exec();
        if (msgBox.clickedButton() == overwriteButton) {
            std::filesystem::remove_all(saveDir);
            break;
        } else if (msgBox.clickedButton() == autoIncrementButton) {
            std::string incremented = incrementDirectoryName(saveDir.string());
            directoryLineEdit_->setText(QString::fromStdString(incremented));
            saveDir = saveDirectory_->getDirectory();
        } else {
            return;
        }
    }

    // Toggle GUI buttons
    recordButton_->setEnabled(false);
    stopButton_->setEnabled(true);
    muscleImagingCheckBox_->setEnabled(false);

    // Parse and set experiment protocol
    std::deque<OperationStep> opSequence;
    int numStepsParsed = parseProtocolString(
        experimentProtocol_->toPlainText().toStdString(), opSequence);
    spdlog::info("Parsed {} protocol steps", opSequence.size());
    if (numStepsParsed < 0) {
        std::string errorMessage = "Invalid experiment protocol string";
        spdlog::error(errorMessage);
        QMessageBox::critical(nullptr, "Error", errorMessage.c_str());
        return;
    } else if (numStepsParsed == 0) {
        spdlog::info("GUI starting recording without any protocol steps");
        programmedRecordingStop_->numBehaviorFramesExpected = -1;
        programmedRecordingStop_->numMuscleFramesExpected = -1;
    } else {
        spdlog::info(
            "GUI starting recording with {} protocol steps", opSequence.size());
        programmedRecordingStop_->numBehaviorFramesExpected =
            opSequence.back().frameIdx;
        programmedRecordingStop_->numMuscleFramesExpected =
            opSequence.back().frameIdx / syncRatioSpinBox_->value();
        spdlog::info(
            "Setting expected number of steps to {} (behavior) and {} (muscle)",
            programmedRecordingStop_->numBehaviorFramesExpected,
            programmedRecordingStop_->numMuscleFramesExpected);
    }
    // An empty opSequence is an open recording; a non-empty one is scheduled.
    currentRecordingIsScheduled_ = numStepsParsed > 0;

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
    spdlog::info(
        "Saved behavior calibration parameters to '{}'",
        behaviorCalibrationFilePath.string());
    if (muscleCamCalibrationParams_.isDefined) {
        std::filesystem::path muscleCalibrationFilePath =
            saveDirectory_->getDirectory() /
            "metadata/calibration_parameters_muscle.yaml";
        muscleCamCalibrationParams_.saveToFile(muscleCalibrationFilePath);
        spdlog::info(
            "Saved muscle calibration parameters to '{}'",
            muscleCalibrationFilePath.string());
    } else {
        spdlog::warn(
            "Muscle camera calibration parameters not defined. Not saving.");
    }

    // Save timing metadata
    if (dualRecordingConfigForSaving_->isRecordingBoth()) {
        std::filesystem::path behaviorCalibrationFilePath =
            saveDirectory_->getDirectory() /
            "metadata/dual_recording_timing.yaml";
        dualRecordingConfigForSaving_->saveToFile(behaviorCalibrationFilePath);
        spdlog::info(
            "Saved dual recording timing parameters to '{}'",
            behaviorCalibrationFilePath.string());
    }

    // Send triggering parameters and start recording. The controller reverts to
    // the streaming (revert-to) params when the recording ends.
    TriggerParams recParams = buildRecordingParams();
    TriggerParams revertToParams = buildStreamingParams();
    arduinoCommunication_->startRecording(recParams, revertToParams, opSequence);

    // The controller waits camFlushTimeUs after START_RECORDING before it starts
    // triggering, so that frames acquired with the previous (streaming) params
    // drain out of the camera buffers. Ignore frames for a fraction of that
    // window on this side too, so the recording does not begin with stale frames
    // (see camFlushTimeUs in comm_protocol/protocol.h).
    std::this_thread::sleep_for(
        std::chrono::microseconds(camFlushTimeUs * 8 / 10));
    programState_->isRecording.store(true);
}

void MainGUIWindow::stopRecording() {
    // Slot for the Stop button: a user-initiated stop.
    endRecording(/*reachedProgrammedEnd=*/false);
}

void MainGUIWindow::endRecording(bool reachedProgrammedEnd) {
    recordButton_->setEnabled(true);
    stopButton_->setEnabled(false);
    muscleImagingCheckBox_->setEnabled(true);

    if (reachedProgrammedEnd) {
        // A scheduled recording reached its end: the controller already reverted
        // to the streaming params via its opSequence STOP step, so there is
        // nothing to send (a STOP_RECORDING here would fault the controller).
    } else if (currentRecordingIsScheduled_) {
        // Manual early abort of a scheduled recording. STOP_RECORDING is only
        // valid for an open recording, so revert by re-streaming instead.
        arduinoCommunication_->stream(buildStreamingParams());
    } else {
        // Open recording: STOP_RECORDING reverts the controller to streaming
        // (using the revert-to params sent with START_RECORDING).
        arduinoCommunication_->stopRecording();
    }

    // Stop queuing frames. The acquirer threads flush any partial behavior
    // group and discard subsequent frames (see behaviorImageAcquirer).
    programState_->isRecording.store(false);
    currentRecordingIsScheduled_ = false;
}

TriggerParams MainGUIWindow::buildStreamingParams() const {
    TriggerParams params;
    // Muscle imaging off => the controller free-runs the behavior camera and
    // never pulses the blue excitation LED (enableMuscle in the protocol). The
    // muscle-only fields below are still sent but ignored in that case.
    params.enableMuscle = muscleImagingEnabled_;
    params.behFrameRate = streamingBehaviorFPS_;
    params.behMuscSyncRatio = streamingSyncRatio_;
    params.behExpTime = static_cast<unsigned int>(
        behaviorExposureTimeSpinBox_->value() * 1000);
    params.muscEffExpTime =
        static_cast<unsigned int>(muscleLightOnTimeSpinBox_->value() * 1000);
    params.pcoCamRollingTime = pcoCamRollingTimeUs_;
    params.pcoCamReadoutTime = pcoCamReadoutTimeUs_;
    return params;
}

TriggerParams MainGUIWindow::buildRecordingParams() const {
    TriggerParams params;
    // The muscle camera is recorded (and the blue excitation light pulsed) only
    // when the dual-recording config has both cameras enabled; otherwise the
    // controller free-runs the behavior camera (enableMuscle = false).
    params.enableMuscle = dualRecordingConfigForSaving_->isRecordingBoth();
    params.behFrameRate = behaviorFPSSpinBox_->value();
    params.behMuscSyncRatio = dualRecordingConfigForSaving_->getSyncRatio();
    params.behExpTime = static_cast<unsigned int>(
        behaviorExposureTimeSpinBox_->value() * 1000);
    params.muscEffExpTime =
        static_cast<unsigned int>(muscleLightOnTimeSpinBox_->value() * 1000);
    params.pcoCamRollingTime = pcoCamRollingTimeUs_;
    params.pcoCamReadoutTime = pcoCamReadoutTimeUs_;
    return params;
}

void MainGUIWindow::closeEvent(QCloseEvent *event) {
    spdlog::info("User is closing GUI window. Quitting gracefully.");
    if (!quitProgram()) {
        event->ignore();
        return;
    }
    event->accept();
}

void MainGUIWindow::browseDirectory() {
    std::string currentDirectory = saveDirectory_->getDirectory();
    QString dir = QFileDialog::getExistingDirectory(
        this,
        "Open Directory",
        QString::fromStdString(currentDirectory),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) {
        directoryLineEdit_->setText(dir);

        saveDirectory_->setDirectory(dir.toStdString());
        spdlog::info("Directory changed to '{}'", dir.toStdString());
    } else {
        spdlog::error("Directory is an empty string; failed to open.");
    }
}

void MainGUIWindow::incrementDirectory() {
    std::string current = saveDirectory_->getDirectory().string();
    std::string incremented = incrementDirectoryName(current);
    directoryLineEdit_->setText(QString::fromStdString(incremented));
}

cv::Mat addCornerMarker(
    const cv::Mat &image,
    double arenaSizeXMm,
    double arenaSizeYMm,
    MotionStagePosition stagePosition,
    const CalibrationParams &behaviorCamCalibrationParams) {
    cv::Mat imageForDisplay = image.clone();
    assert(imageForDisplay.size() == image.size());

    std::vector<std::tuple<double, double>> cornerPositions = {
        {0.0, 0.0},
        {arenaSizeXMm, 0.0},
        {arenaSizeXMm, arenaSizeYMm},
        {0.0, arenaSizeYMm}};

    std::vector<cv::Point> pixelPoints;
    for (auto [x, y] : cornerPositions) {
        int pixelRow, pixelCol;
        std::tie(pixelRow, pixelCol) =
            behaviorCamCalibrationParams.stagePosAndPhysicalPosToPixelPos(
                stagePosition.xPosMm, stagePosition.yPosMm, x, y);
        pixelPoints.emplace_back(pixelCol, pixelRow);
        cv::circle(
            imageForDisplay,
            cv::Point(pixelCol, pixelRow),
            5,
            cv::Scalar(255, 255, 255),
            -1);
    }
    for (size_t i = 0; i < pixelPoints.size(); ++i) {
        cv::line(
            imageForDisplay,
            pixelPoints[i],
            pixelPoints[(i + 1) % pixelPoints.size()],
            cv::Scalar(255, 0, 0),
            2);
    }

    return imageForDisplay;
}

void MainGUIWindow::updateBehaviorImageDisplay() {
    cv::Mat latestFrame =
        behaviorRecordingState_->latestFrameHolder->getLatestFrameData().image;
    if (latestFrame.empty()) {
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

    // Warp the active-area mask into camera-image space and convert the
    // grayscale frame to BGR and tint out-of-arena pixels red at 50% opacity
    // for visualization.
    cv::Mat activeMaskCurrView =
        activeAreaMask_.warpToCurrentView(correctedFrame, myStagePosition);
    cv::Mat bgrImage;
    cv::cvtColor(correctedFrame, bgrImage, cv::COLOR_GRAY2BGR);
    cv::Mat outsideArena;
    cv::threshold(
        activeMaskCurrView, outsideArena, 0, 255, cv::THRESH_BINARY_INV);
    std::vector<cv::Mat> channels(3);
    cv::split(bgrImage, channels);
    // Red tint at 50% opacity: new_red = curr + (255 - curr) / 2
    cv::Mat inv, halfInv, tintedRed;
    cv::subtract(cv::Scalar(255), channels[2], inv);
    cv::divide(inv, 2, halfInv);
    cv::add(channels[2], halfInv, tintedRed);
    tintedRed.copyTo(channels[2], outsideArena); // red channel (BGR)
    cv::merge(channels, bgrImage);

    cv::Mat imageForDisplay = addCornerMarker(
        bgrImage,
        activeAreaMask_.arenaWidthMm,
        activeAreaMask_.arenaHeightMm,
        myStagePosition,
        behaviorCamCalibrationParams_);

    QImage qImage = cvMatToQImage(imageForDisplay);
    QPixmap pixmap = QPixmap::fromImage(qImage).scaled(
        behaviorImageDisplayLabel_->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation);
    behaviorImageDisplayLabel_->setPixmap(pixmap);
}

void MainGUIWindow::updateMuscleImageDisplay() {
    if (!muscleImagingEnabled_) {
        muscleImageDisplayLabel_->clear();
        return;
    }

    cv::Mat latestFrame =
        muscleRecordingState_->latestFrameHolder->getLatestFrameData().image;
    if (latestFrame.empty()) {
        return;
    }
    cv::Mat processedFrame;
    convert16BitTo8Bit(
        latestFrame,
        processedFrame,
        muscleImage16To8BitScale_,
        muscleImage16To8BitOffset_);
    reorientMuscleImage(processedFrame, processedFrame);
    QImage qImage = cvMatToQImage(processedFrame);
    QPixmap pixmap = QPixmap::fromImage(qImage).scaled(
        muscleImageDisplayLabel_->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation);
    muscleImageDisplayLabel_->setPixmap(pixmap);
}

DualRecordingConfigWindow::DualRecordingConfigWindow(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<DualRecordingConfig> dualRecordingConfig,
    const MuscleCameraROI &muscleCameraROI,
    QWidget *parent)
    : QDialog(parent), dualRecordingConfigForSaving_(dualRecordingConfig),
      recorderConfig_(recorderConfig), muscleCameraROI_(muscleCameraROI) {
    setWindowTitle("Dual recording configuration");
    mainLayout_ = new QVBoxLayout(this);
    resize(desiredWidth_, desiredHeight_);

    // Block for recording behavior only
    QLabel *labelTitle =
        new QLabel("<b>Recording both behavior and muscle</b>");
    QLabel *labelNoMuscle = new QLabel(
        "<i>If you wish to record behavior only</i>, click the button below.");
    labelNoMuscle->setWordWrap(true);
    withoutMuscleButton_ = new QPushButton("Record behavior only", this);
    connect(
        withoutMuscleButton_,
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
    QLabel *behaviorCameraFPSLabel =
        new QLabel("Behavior camera FPS (Hz)", this);
    behaviorCameraFPSSpinBox_ = new QSpinBox(this);
    behaviorCameraFPSSpinBox_->setRange(1, 1000);
    behaviorCameraFPSSpinBox_->setValue(recorderConfig.getParameter<int>(
        "behavior_camera", "default_recording_fps"));
    behaviorCameraFPSLayout->addWidget(behaviorCameraFPSLabel);
    behaviorCameraFPSLayout->addWidget(behaviorCameraFPSSpinBox_);
    mainLayout_->addLayout(behaviorCameraFPSLayout);

    // Behavior-muscle sync ratio
    QHBoxLayout *syncRatioLayout = new QHBoxLayout();
    QLabel *syncRatioLabel =
        new QLabel("Sync ratio (behavior FPS : muscle FPS)", this);
    syncRatioSpinBox_ = new QSpinBox(this);
    syncRatioSpinBox_->setRange(1, 100);
    syncRatioSpinBox_->setValue(recorderConfig.getParameter<int>(
        "muscle_camera", "default_recording_sync_ratio"));
    syncRatioLayout->addWidget(syncRatioLabel);
    syncRatioLayout->addWidget(syncRatioSpinBox_);
    mainLayout_->addLayout(syncRatioLayout);

    // Muscle exposure time
    QHBoxLayout *muscleLightOnTimeLayout = new QHBoxLayout();
    QLabel *muscleLightOnTimeLabel =
        new QLabel("Muscle exposure (light-on) time (ms)", this);
    muscleLightOnTimeSpinBox_ = new QDoubleSpinBox(this);
    muscleLightOnTimeSpinBox_->setRange(0.001, 10000.0);
    muscleLightOnTimeSpinBox_->setValue(
        recorderConfig.getParameter<int>(
            "muscle_camera", "default_light_on_time_us") /
        1000.0);
    muscleLightOnTimeLayout->addWidget(muscleLightOnTimeLabel);
    muscleLightOnTimeLayout->addWidget(muscleLightOnTimeSpinBox_);
    mainLayout_->addLayout(muscleLightOnTimeLayout);

    withMuscleButton_ = new QPushButton("Record behavior and muscle", this);
    connect(
        withMuscleButton_,
        &QPushButton::clicked,
        this,
        &DualRecordingConfigWindow::onButtonClicked);
    mainLayout_->addWidget(withMuscleButton_);
}

DualRecordingConfigWindow::~DualRecordingConfigWindow() {
    // Nothing to clean up here, as we are using Qt's parent-child
}

void DualRecordingConfigWindow::onButtonClicked() {
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

    bool isValid = dualRecordingConfigForSaving_->computeParameters(
        muscleImageHeight,
        muscleCameraLineScanTimeUs,
        muscleCameraReadoutTimeUs);

    if (!isValid) {
        QMessageBox::critical(
            this,
            "Error",
            "Invalid configuration for recording both behavior and muscle. "
            "Please check the parameters and try again. "
            "In particular, check if the muscle recording interval "
            "(i.e. 1 / muscleFPS) is greater than the minimum. See "
            "https://github.com/NeLy-EPFL/spotlight-control/issues/79.");
    } else {
        spdlog::info(
            "Behavior-muscle corecording config set: "
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
    std::deque<OperationStep> &opSequence)
/**
 * Parse the experiment-protocol text field into an opSequence. The text is a
 * ";"-separated list of steps, each "frameIdx/channel/op":
 *   - "<n>/ch2/on", "<n>/ch3/off": toggle an optogenetics channel
 *   - "<n>/x/stop": end the recording and revert to streaming
 * A single ";" denotes an empty (open) recording. Returns the number of steps,
 * or -1 on a malformed string.
 */
{
    opSequence.clear();

    auto reportError = []() {
        std::string errorMessage = "Invalid experiment protocol";
        spdlog::error(errorMessage);
        QMessageBox::critical(nullptr, "Error", errorMessage.c_str());
        return -1;
    };

    if (protocolTextFieldString == ";") {
        return 0;
    }

    std::istringstream stream(protocolTextFieldString);
    std::string token;
    int numStepsParsed = 0;
    while (std::getline(stream, token, ';')) {
        if (token.empty()) {
            return reportError();
        }

        std::istringstream tokenStream(token);
        std::string frameStr, channelStr, opStr;
        if (!std::getline(tokenStream, frameStr, '/') ||
            !std::getline(tokenStream, channelStr, '/') ||
            !std::getline(tokenStream, opStr, '/')) {
            return reportError();
        }

        unsigned long frameIdx = 0;
        OptoChannel channel = OptoChannel::ALL;
        OpType op = OpType::STOP;
        try {
            frameIdx = std::stoul(frameStr);
            if (channelStr == "x" && opStr == "stop") {
                channel = OptoChannel::ALL;
                op = OpType::STOP;
            } else if (channelStr.rfind("ch", 0) == 0) {
                channel =
                    static_cast<OptoChannel>(std::stoi(channelStr.substr(2)));
                if (opStr == "on") {
                    op = OpType::ON;
                } else if (opStr == "off") {
                    op = OpType::OFF;
                } else {
                    return reportError();
                }
            } else {
                return reportError();
            }
        } catch (const std::exception &) {
            return reportError();
        }

        OperationStep step(frameIdx, channel, op);
        if (!step.isValid) {
            return reportError();
        }
        opSequence.push_back(step);
        ++numStepsParsed;
    }

    return numStepsParsed;
}
