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
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<TrackingControlState> trackingControlState,
    CalibrationParams &behaviorCamCalibrationParams,
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
      saveDirectory_(saveDirectory),
      arduinoCommunication_(arduinoCommunication), programState_(programState),
      programmedRecordingStop_(programmedRecordingStop),
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

    // The muscle camera is started (and waited for) in run_spotlight_main before
    // this window is constructed, so it is ready by now.

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
    // Don't connect to ArduinoCommunication! This value is only used during
    // recording. When streaming, the behavior frame rate comes from
    // behavior_camera/streaming_frame_rate and this field is ignored.
    QHBoxLayout *behaviorFPSLayout = new QHBoxLayout();
    behaviorFPSLayout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behaviorFPSLayout->addWidget(behaviorFPSSpinBox_);

    // Behavior-muscle synchronization ratio
    syncRatioSpinBox_ = new QSpinBox(this);
    syncRatioSpinBox_->setRange(1, INT_MAX);
    int syncRatio = recorderConfig.getParameter<int>(
        "muscle_camera", "default_recording_sync_ratio");
    syncRatioSpinBox_->setValue(syncRatio);
    // Muscle-only parameter: disabled unless muscle imaging is enabled (the
    // checkbox below toggles it). Don't connect to ArduinoCommunication! This
    // value is only used during recording. When streaming, the sync ratio comes
    // from muscle_camera/streaming_sync_ratio and this field is ignored.
    syncRatioSpinBox_->setEnabled(false);
    QHBoxLayout *syncRatioLayout = new QHBoxLayout();
    syncRatioLayout->addWidget(new QLabel("Behavior FPS : muscle FPS"));
    syncRatioLayout->addWidget(syncRatioSpinBox_);

    // Behavior exposure time widget
    behaviorExposureTimeSpinBox_ = new QDoubleSpinBox(this);
    behaviorExposureTimeSpinBox_->setRange(0.001, 1000.0);
    defaultBehExpTimeUs_ = recorderConfig.getParameter<int>(
        "behavior_camera", "default_exposure_time_us");
    behaviorExposureTimeSpinBox_->setValue(defaultBehExpTimeUs_ / 1000.0);
    // Buffered recording parameter: applied only when a recording starts (see
    // buildRecordingParams). During streaming the controller always runs the
    // default behavior exposure (see buildStreamingParams), so editing this spin
    // box has no live effect and is intentionally not re-streamed -- the
    // controller's status display must show the actual live behavior, not the
    // not-yet-executed recording config.
    QHBoxLayout *behaviorExposureTimeLayout = new QHBoxLayout();
    behaviorExposureTimeLayout->addWidget(
        new QLabel("Behavior exposure time (ms)"));
    behaviorExposureTimeLayout->addWidget(behaviorExposureTimeSpinBox_);

    // Muscle exposure time widget
    muscleLightOnTimeSpinBox_ = new QDoubleSpinBox(this);
    muscleLightOnTimeSpinBox_->setRange(0.001, 1000.0);
    defaultMuscLightOnTimeUs_ = recorderConfig.getParameter<int>(
        "muscle_camera", "default_light_on_time_us");
    muscleLightOnTimeSpinBox_->setValue(defaultMuscLightOnTimeUs_ / 1000.0);
    // Initialize the free-running (auto-sequence) muscle camera to the streaming
    // muscle frame rate. In continuous mode the nominal exposure sets the frame
    // rate; it is switched to the recording rate when a recording starts and back
    // when it ends (see startRecording/endRecording).
    pushMuscleCameraExposure(streamingBehaviorFPS_, streamingSyncRatio_);
    // Muscle-only parameter: disabled unless muscle imaging is enabled (the
    // checkbox below toggles it). Like the behavior exposure, this is a buffered
    // recording parameter -- applied only when a recording starts; during
    // streaming the controller always runs the default light-on time, so editing
    // it has no live effect and is not re-streamed.
    muscleLightOnTimeSpinBox_->setEnabled(false);
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
            bool enabled = (state == Qt::Checked);
            muscleImagingEnabled_ = enabled;
            spdlog::info(
                enabled ? "Enabling muscle imaging" : "Disabling muscle imaging");
            // The muscle-only parameters are editable only when imaging muscle.
            syncRatioSpinBox_->setEnabled(enabled);
            muscleLightOnTimeSpinBox_->setEnabled(enabled);
            // Re-stream so the controller switches modes immediately (sending a
            // STREAM mid-recording would revert the controller and abort it).
            if (!programState_->isRecording.load()) {
                arduinoCommunication_->stream(buildStreamingParams());
            }
        });
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

    // Add timer to keep checking for programmedRecordingStop. The acquirer
    // threads stop recording exactly on the programmed frame count by
    // themselves; this only finalizes the GUI (reverts the UI and cameras to
    // streaming) shortly after.
    QTimer *programmedStopCheckTimer = new QTimer(this);
    connect(
        programmedStopCheckTimer,
        &QTimer::timeout,
        this,
        [this, programmedRecordingStop]() {
            if (programmedRecordingStop->programmedStopReached.load()) {
                spdlog::info("Protocol stop reached. Finalizing recording.");
                endRecording(/*reachedProgrammedEnd=*/true);
                programmedRecordingStop->numBehaviorFramesExpected = -1;
                programmedRecordingStop->numMuscleFramesExpected = -1;
                programmedRecordingStop->programmedStopReached.store(
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

bool MainGUIWindow::validateAndPrepareRecording(
    std::deque<OperationStep> &opSequence,
    int &muscleNominalExposureUs,
    int &muscleBufferTimeUs) {
    muscleNominalExposureUs = 0;
    muscleBufferTimeUs = 0;

    // Parse the experiment protocol into an opSequence and record the
    // programmed-stop frame counts. A malformed string aborts the recording.
    int numStepsParsed = parseProtocolString(
        experimentProtocol_->toPlainText().toStdString(), opSequence);
    spdlog::info("Parsed {} protocol steps", opSequence.size());
    if (numStepsParsed < 0) {
        // parseProtocolString has already shown a detailed error dialog.
        return false;
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

    // When imaging muscle, derive and validate the continuous-mode muscle
    // trigger timing from the current recording parameters.
    if (muscleImagingCheckBox_->isChecked()) {
        MuscleTriggerTiming muscleTriggerTiming(
            behaviorFPSSpinBox_->value(),
            syncRatioSpinBox_->value(),
            static_cast<int>(muscleLightOnTimeSpinBox_->value() * 1000));
        double rollingShutterLineTimeUs = recorderConfig_.getParameter<double>(
            "muscle_camera", "rolling_shutter_line_time_us");
        int muscleCamReadoutTimeUs = recorderConfig_.getParameter<double>(
            "muscle_camera", "sensor_readout_time_us");
        if (!muscleTriggerTiming.computeParameters(
                muscleRecordingState_->muscleCamera->getNumLinesScanned(),
                rollingShutterLineTimeUs,
                muscleCamReadoutTimeUs)) {
            QMessageBox::critical(
                this,
                "Error",
                "Invalid muscle recording configuration. In particular, check "
                "that the muscle recording interval (1 / muscle FPS) is long "
                "enough for the rolling shutter, light-on, and sensor readout "
                "times. See "
                "https://github.com/NeLy-EPFL/spotlight-control/issues/79.");
            return false;
        }
        muscleNominalExposureUs = muscleTriggerTiming.getNominalExposureUs();
        muscleBufferTimeUs = muscleTriggerTiming.getBufferTimeUs();
    }

    return true;
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
            // Keep incrementing until we land on a free (non-existent or
            // empty) directory, in case the immediately-next number is also
            // already taken.
            saveDir = incrementDirectoryName(saveDir.string());
            while (std::filesystem::is_directory(saveDir) &&
                   !std::filesystem::is_empty(saveDir)) {
                saveDir = incrementDirectoryName(saveDir.string());
            }
            directoryLineEdit_->setText(
                QString::fromStdString(saveDir.string()));
            saveDir = saveDirectory_->getDirectory();
        } else {
            return;
        }
    }

    // Validate the experiment protocol and (when imaging muscle) the muscle
    // trigger timing before any side effects -- camera exposure change, button
    // toggles, directory creation, metadata writes -- so an invalid
    // configuration aborts cleanly and leaves nothing behind. The derived muscle
    // timing is returned for the metadata and the exposure update below.
    std::deque<OperationStep> opSequence;
    int muscleNominalExposureUs = 0;
    int muscleBufferTimeUs = 0;
    if (!validateAndPrepareRecording(
            opSequence, muscleNominalExposureUs, muscleBufferTimeUs)) {
        return;
    }

    // Switch the free-running camera to the recording muscle frame rate before
    // START_RECORDING, so it is already emitting common-time onsets at the
    // recording cadence when the firmware begins locking the behavior frames to
    // them. The controller's camFlushTimeUs delay covers the transient while the
    // new exposure takes effect.
    if (muscleImagingCheckBox_->isChecked()) {
        muscleRecordingState_->muscleCamera->setNominalExposureUs(
            static_cast<unsigned int>(muscleNominalExposureUs));
    }

    // Toggle GUI buttons
    recordButton_->setEnabled(false);
    stopButton_->setEnabled(true);
    muscleImagingCheckBox_->setEnabled(false);

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
        // Convert ms to us
        static_cast<int>(behaviorExposureTimeSpinBox_->value() * 1000),
        static_cast<int>(muscleLightOnTimeSpinBox_->value() * 1000),
        muscleNominalExposureUs,
        muscleBufferTimeUs,
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

    // Revert the free-running muscle camera to the streaming muscle frame rate,
    // matching the streaming params the controller was just reverted to.
    pushMuscleCameraExposure(streamingBehaviorFPS_, streamingSyncRatio_);

    // Stop queuing frames. The acquirer threads flush any partial behavior
    // group and discard subsequent frames (see behaviorImageAcquirer).
    programState_->isRecording.store(false);
    currentRecordingIsScheduled_ = false;
}

void MainGUIWindow::pushMuscleCameraExposure(int behFrameRate, int syncRatio) {
    // In continuous (auto-sequence) mode the camera free-runs at
    // 1/(nominalExposure + readout), so the nominal per-line exposure sets the
    // frame rate. Pick it so the camera produces muscle frames at
    // behFrameRate / syncRatio. See docs/data_acquisition.md and
    // MuscleTriggerTiming.
    unsigned int muscleIntervalUs =
        static_cast<unsigned int>(1000000.0 * syncRatio / behFrameRate);
    int exposureUs = static_cast<int>(muscleIntervalUs) -
                     static_cast<int>(pcoCamReadoutTimeUs_);
    if (exposureUs <= 0) {
        spdlog::error(
            "Cannot set muscle camera exposure: muscle interval ({} us) is not "
            "longer than the sensor readout time ({} us).",
            muscleIntervalUs,
            pcoCamReadoutTimeUs_);
        return;
    }
    muscleRecordingState_->muscleCamera->setNominalExposureUs(
        static_cast<unsigned int>(exposureUs));
}

TriggerParams MainGUIWindow::buildStreamingParams() const {
    TriggerParams params;
    // During streaming the controller always runs DEFAULT parameters. The
    // recording spin boxes (behavior FPS, sync ratio, behavior exposure, muscle
    // light-on) are buffered in the GUI and take effect only when a recording
    // starts (see buildRecordingParams). This keeps the controller's status
    // display showing the actual live behavior, never the not-yet-executed
    // recording config.
    //
    // The one live streaming control is the muscle-imaging checkbox: it toggles
    // whether the behavior camera is locked to the muscle camera and the blue
    // excitation LED is pulsed (enableMuscle), for muscle preview. When false,
    // the muscle-only fields are still sent but ignored by the controller.
    params.enableMuscle = muscleImagingEnabled_;
    params.behFrameRate = streamingBehaviorFPS_;
    params.behMuscSyncRatio = streamingSyncRatio_;
    params.behExpTime = static_cast<unsigned int>(defaultBehExpTimeUs_);
    params.muscEffExpTime = static_cast<unsigned int>(defaultMuscLightOnTimeUs_);
    params.pcoCamRollingTime = pcoCamRollingTimeUs_;
    params.pcoCamReadoutTime = pcoCamReadoutTimeUs_;
    return params;
}

TriggerParams MainGUIWindow::buildRecordingParams() const {
    TriggerParams params;
    // The muscle camera is recorded (and the blue excitation light pulsed) only
    // when muscle imaging is enabled; otherwise the controller free-runs the
    // behavior camera (enableMuscle = false).
    params.enableMuscle = muscleImagingEnabled_;
    params.behFrameRate = behaviorFPSSpinBox_->value();
    params.behMuscSyncRatio = syncRatioSpinBox_->value();
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

int parseProtocolString(
    const std::string &protocolTextFieldString,
    std::deque<OperationStep> &opSequence)
/**
 * Parse the experiment-protocol text field into an opSequence. The text is a
 * ";"-separated list of steps, each "frameIdx/channel/op":
 *   - "<n>/ch2/on", "<n>/ch3/off": toggle an optogenetics channel
 *   - "<n>/x/stop": end the recording and revert to streaming
 * An empty string (or a single ";") denotes an open recording with no
 * programmed steps. Returns the number of steps, or -1 on a malformed string.
 */
{
    opSequence.clear();

    auto reportError = []() {
        spdlog::error("Invalid experiment protocol string");
        QMessageBox::critical(
            nullptr,
            "Invalid experiment protocol",
            "The experiment protocol string is invalid.\n\n"
            "It must be a ';'-separated list of steps, each of the form\n"
            "    frameIdx/channel/op\n"
            "where:\n"
            "  - frameIdx is a non-negative integer: the behavior-frame index "
            "after which the step is applied;\n"
            "  - to switch an optogenetics channel, channel is 'ch2' or 'ch3' "
            "(channel 1 is reserved for the IR LED) and op is 'on' or 'off';\n"
            "  - to end the recording, channel is 'x' and op is 'stop'.\n\n"
            "Leave empty for open recording (no programmed stop).\n\n"
            "Example:\n"
            "    300/ch2/on;600/ch2/off;900/x/stop\n"
            "turns channel 2 on after frame 300, off after frame 600, and "
            "stops the recording after frame 900.");
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
