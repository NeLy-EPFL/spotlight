#include "recorder/apps/gui.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace {
// Layout constants for the muscle histogram + range slider widget.
constexpr int kHistogramNumBins = 256;
constexpr int kHistogramWidgetHeight = 65;
constexpr int kSliderAreaHeight = 12;
constexpr int kHandleHalfWidth = 5;

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

MuscleHistogramWidget::MuscleHistogramWidget(
    int histogramMin,
    int histogramMax,
    int defaultVmin,
    int defaultVmax,
    QWidget *parent)
    : QWidget(parent), histogramMin_(histogramMin), histogramMax_(histogramMax),
      vmin_(defaultVmin), vmax_(defaultVmax),
      histogram_(kHistogramNumBins, 0.0f) {
    setMinimumHeight(kHistogramWidgetHeight);
}

void MuscleHistogramWidget::setImage(const cv::Mat &image16Bit) {
    if (image16Bit.empty()) {
        return;
    }
    int numBins = static_cast<int>(histogram_.size());
    int channels[] = {0};
    int histSize[] = {numBins};
    // calcHist's upper bound is exclusive, so add 1 to include histogramMax_.
    float valueRange[] = {
        static_cast<float>(histogramMin_),
        static_cast<float>(histogramMax_ + 1)};
    const float *ranges[] = {valueRange};
    cv::Mat hist;
    cv::calcHist(
        &image16Bit, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
    // Normalize bin heights to the tallest bin so the histogram fills the
    // available height regardless of frame size / brightness.
    double maxBin = 0.0;
    cv::minMaxLoc(hist, nullptr, &maxBin);
    for (int i = 0; i < numBins; ++i) {
        histogram_[i] =
            maxBin > 0.0 ? hist.at<float>(i) / static_cast<float>(maxBin) : 0.0f;
    }
    update();
}

int MuscleHistogramWidget::valueToX(int value) const {
    int usableWidth = std::max(1, width() - 2 * kHandleHalfWidth);
    return kHandleHalfWidth + (value - histogramMin_) * usableWidth /
                                  std::max(1, histogramMax_ - histogramMin_);
}

int MuscleHistogramWidget::xToValue(int x) const {
    int usableWidth = std::max(1, width() - 2 * kHandleHalfWidth);
    int value = histogramMin_ + (x - kHandleHalfWidth) *
                                    (histogramMax_ - histogramMin_) /
                                    usableWidth;
    return std::clamp(value, histogramMin_, histogramMax_);
}

void MuscleHistogramWidget::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);

    int sliderTop = height() - kSliderAreaHeight;
    int histogramHeight = sliderTop;

    painter.fillRect(rect(), QColor(30, 30, 30));

    // Histogram bars.
    int numBins = static_cast<int>(histogram_.size());
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(180, 180, 180));
    for (int i = 0; i < numBins; ++i) {
        int x0 = width() * i / numBins;
        int x1 = width() * (i + 1) / numBins;
        int barHeight = static_cast<int>(histogram_[i] * histogramHeight);
        painter.drawRect(
            x0, histogramHeight - barHeight, std::max(1, x1 - x0), barHeight);
    }

    int xMin = valueToX(vmin_);
    int xMax = valueToX(vmax_);

    // Dim the regions outside the selected [vmin, vmax] window.
    painter.setBrush(QColor(0, 0, 0, 130));
    painter.drawRect(0, 0, xMin, histogramHeight);
    painter.drawRect(xMax, 0, width() - xMax, histogramHeight);

    // Slider groove and selected span.
    int grooveY = sliderTop + kSliderAreaHeight / 2;
    painter.setPen(QPen(QColor(120, 120, 120), 2));
    painter.drawLine(
        kHandleHalfWidth, grooveY, width() - kHandleHalfWidth, grooveY);
    painter.setPen(QPen(QColor(80, 160, 240), 3));
    painter.drawLine(xMin, grooveY, xMax, grooveY);

    // Min handle (blue) with a guide line over the histogram.
    painter.setPen(QPen(QColor(80, 160, 240), 1));
    painter.drawLine(xMin, 0, xMin, histogramHeight);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(80, 160, 240));
    painter.drawRect(
        xMin - kHandleHalfWidth,
        sliderTop,
        2 * kHandleHalfWidth,
        kSliderAreaHeight);

    // Max handle (orange) with a guide line over the histogram.
    painter.setPen(QPen(QColor(240, 160, 60), 1));
    painter.drawLine(xMax, 0, xMax, histogramHeight);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(240, 160, 60));
    painter.drawRect(
        xMax - kHandleHalfWidth,
        sliderTop,
        2 * kHandleHalfWidth,
        kSliderAreaHeight);

    // Value labels.
    painter.setPen(Qt::white);
    painter.drawText(
        QRect(2, 0, width() - 4, 14),
        Qt::AlignLeft,
        QString("min %1").arg(vmin_));
    painter.drawText(
        QRect(2, 0, width() - 4, 14),
        Qt::AlignRight,
        QString("max %1").arg(vmax_));
}

void MuscleHistogramWidget::mousePressEvent(QMouseEvent *event) {
    int x = static_cast<int>(event->position().x());
    // Grab whichever handle is closer to the click.
    draggedHandle_ = std::abs(x - valueToX(vmin_)) <= std::abs(x - valueToX(vmax_))
                         ? DraggedHandle::Min
                         : DraggedHandle::Max;
    mouseMoveEvent(event);
}

void MuscleHistogramWidget::mouseMoveEvent(QMouseEvent *event) {
    if (draggedHandle_ == DraggedHandle::None) {
        return;
    }
    int value = xToValue(static_cast<int>(event->position().x()));
    if (draggedHandle_ == DraggedHandle::Min) {
        // The min handle can never move past the max handle.
        vmin_ = std::min(value, vmax_);
    } else {
        vmax_ = std::max(value, vmin_);
    }
    update();
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
    // Stage bounds are derived externally (in run_spotlight_main) from the
    // arena dimensions and the fitted calibration model, then passed in here.
    minXAbsoluteMm_ = minXAbsoluteMm;
    maxXAbsoluteMm_ = maxXAbsoluteMm;
    minYAbsoluteMm_ = minYAbsoluteMm;
    maxYAbsoluteMm_ = maxYAbsoluteMm;

    int guiMotionStagePreviewUpdateFreq = recorderConfig.getParameter<int>(
        "gui", "motion_stage_preview_update_frequency_hz");
    int guiMotionStagePreviewHeight =
        recorderConfig.getParameter<int>("gui", "motion_stage_preview_height");
    // Enlarge the stage preview by 50% over its configured height; the width is
    // derived from the height below, so it scales by the same factor.
    guiMotionStagePreviewHeight = guiMotionStagePreviewHeight * 3 / 2;

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
    loadRecordingParameters();

    // Arrange layout. The config text boxes (and their labels) occupy a
    // fixed-width column on the left, wide enough that the labels and spin boxes
    // are not cramped. The record/stop buttons sit immediately to the right of
    // that column with a small gap. The save directory row joins the same column
    // below the protocol box, so the vertical gap above it matches the spacing
    // between the config rows.
    const int configRowsWidth = 700;
    const int buttonsGap = 12;

    QVBoxLayout *configRowsLayout = new QVBoxLayout();
    configRowsLayout->setContentsMargins(0, 0, 0, 0);
    configRowsLayout->addLayout(createBehaviorFPSRow());
    configRowsLayout->addLayout(createSyncRatioRow());
    configRowsLayout->addLayout(createBehaviorExposureRow());
    configRowsLayout->addLayout(createMuscleExposureRow());
    configRowsLayout->addLayout(createProtocolRow());
    QWidget *configRowsPanel = new QWidget(this);
    configRowsPanel->setLayout(configRowsLayout);
    configRowsPanel->setFixedWidth(configRowsWidth);

    // Config rows on the left, the buttons immediately to their right (with a
    // gap), and a trailing stretch so the buttons stay next to the boxes rather
    // than being pushed to the far edge.
    QHBoxLayout *configAndButtonsLayout = new QHBoxLayout();
    configAndButtonsLayout->setContentsMargins(0, 0, 0, 0);
    configAndButtonsLayout->addWidget(configRowsPanel);
    configAndButtonsLayout->addSpacing(buttonsGap);
    configAndButtonsLayout->addLayout(createRecordStopButtons());
    configAndButtonsLayout->addStretch();

    // The config-rows-plus-buttons block and the save directory row share one
    // column so the save directory is separated from the protocol box by the
    // same vertical spacing as the config rows are from each other. Both rows
    // start at x = 0 and use the same configRowsWidth + buttonsGap, so the save
    // directory text box aligns with the spin boxes above and the Browse /
    // Increment buttons align with the Record / Stop buttons. The panel is left
    // unconstrained in width so the (horizontally laid out) Browse / Increment
    // buttons are never clipped; the trailing stretch in each row absorbs the
    // slack.
    QVBoxLayout *topBlockLayout = new QVBoxLayout();
    topBlockLayout->setContentsMargins(0, 0, 0, 0);
    topBlockLayout->addLayout(configAndButtonsLayout);
    topBlockLayout->addLayout(createSaveDirectoryRow(configRowsWidth, buttonsGap));
    QWidget *topBlockPanel = new QWidget(this);
    topBlockPanel->setLayout(topBlockLayout);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(topBlockPanel, 0, Qt::AlignLeft);
    layout->addLayout(createLiveImageDisplays());
    setLayout(layout);

    setupProgrammedStopTimer();

    // Start in streaming mode: live preview only, not saving. Muscle imaging is
    // off by default (enableMuscle = false: behavior camera free-runs, blue
    // excitation LED off). The controller is configured with a single STREAM
    // command.
    recordButton_->setEnabled(true);
    stopButton_->setEnabled(false);
    muscleImagingCheckBox_->setEnabled(true);
    programState_->isRecording.store(false);
    arduinoCommunication_->stream(buildStreamingParams());
}

void MainGUIWindow::loadRecordingParameters() {
    streamingBehaviorFPS_ = recorderConfig_.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");

    // Load rolling shutter parameter
    double rollingShutterLineTimeUs = recorderConfig_.getParameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    int muscleCamReadoutTimeUs = recorderConfig_.getParameter<double>(
        "muscle_camera", "sensor_readout_time_us");

    // Load streaming sync ratio
    streamingSyncRatio_ = recorderConfig_.getParameter<int>(
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
}

QLayout *MainGUIWindow::createBehaviorFPSRow() {
    behaviorFPSSpinBox_ = new QSpinBox(this);
    behaviorFPSSpinBox_->setRange(1, 1000);
    int behaviorCameraDefaultRecordingFrameRate =
        recorderConfig_.getParameter<int>(
            "behavior_camera", "default_recording_fps");
    behaviorFPSSpinBox_->setValue(behaviorCameraDefaultRecordingFrameRate);
    // Don't connect to ArduinoCommunication! This value is only used during
    // recording. When streaming, the behavior frame rate comes from
    // behavior_camera/streaming_frame_rate and this field is ignored.
    QHBoxLayout *behaviorFPSLayout = new QHBoxLayout();
    behaviorFPSLayout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behaviorFPSLayout->addWidget(behaviorFPSSpinBox_);
    return behaviorFPSLayout;
}

QLayout *MainGUIWindow::createSyncRatioRow() {
    syncRatioSpinBox_ = new QSpinBox(this);
    syncRatioSpinBox_->setRange(1, INT_MAX);
    int syncRatio = recorderConfig_.getParameter<int>(
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
    return syncRatioLayout;
}

QLayout *MainGUIWindow::createBehaviorExposureRow() {
    behaviorExposureTimeSpinBox_ = new QDoubleSpinBox(this);
    behaviorExposureTimeSpinBox_->setRange(0.001, 1000.0);
    defaultBehExpTimeUs_ = recorderConfig_.getParameter<int>(
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
    return behaviorExposureTimeLayout;
}

QLayout *MainGUIWindow::createMuscleExposureRow() {
    muscleLightOnTimeSpinBox_ = new QDoubleSpinBox(this);
    muscleLightOnTimeSpinBox_->setRange(0.001, 1000.0);
    defaultMuscLightOnTimeUs_ = recorderConfig_.getParameter<int>(
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
    return muscleLightOnTimeLayout;
}

QLayout *MainGUIWindow::createProtocolRow() {
    QLabel *protocolLabel = new QLabel("Experiment protocol", this);
    experimentProtocol_ = new QTextEdit(this);
    experimentProtocol_->setMinimumHeight(40);
    QVBoxLayout *protocolLayout = new QVBoxLayout();
    protocolLayout->addWidget(protocolLabel);
    protocolLayout->addWidget(experimentProtocol_);
    return protocolLayout;
}

QLayout *MainGUIWindow::createSaveDirectoryRow(
    int configRowsWidth, int buttonsGap) {
    directoryLineEdit_ = new QLineEdit(this);
    directoryLineEdit_->setText(saveDirectory_->getDirectory().c_str());
    connect(
        directoryLineEdit_,
        &QLineEdit::textChanged,
        this,
        [this](const QString &text) {
            spdlog::debug("saveDirectory changed to {}", text.toStdString());
            saveDirectory_->setDirectory(text.toStdString());
        });
    QPushButton *browseButton = new QPushButton("Browse", this);
    QPushButton *incrementButton = new QPushButton("Increment", this);

    // The label + text box occupy the same fixed-width column as the config
    // rows, so the text box's right edge aligns with the spin boxes above. The
    // Browse / Increment buttons then sit (after the same gap) left-aligned to
    // the Record / Stop buttons column.
    QHBoxLayout *labelAndEditLayout = new QHBoxLayout();
    labelAndEditLayout->setContentsMargins(0, 0, 0, 0);
    labelAndEditLayout->addWidget(new QLabel("Save directory"));
    labelAndEditLayout->addWidget(directoryLineEdit_);
    QWidget *labelAndEditPanel = new QWidget(this);
    labelAndEditPanel->setLayout(labelAndEditLayout);
    labelAndEditPanel->setFixedWidth(configRowsWidth);

    QHBoxLayout *directoryLayout = new QHBoxLayout();
    directoryLayout->setContentsMargins(0, 0, 0, 0);
    directoryLayout->addWidget(labelAndEditPanel);
    directoryLayout->addSpacing(buttonsGap);
    directoryLayout->addWidget(browseButton);
    directoryLayout->addWidget(incrementButton);
    directoryLayout->addStretch();

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
    return directoryLayout;
}

QLayout *MainGUIWindow::createBehaviorPreviewColumn(
    int previewWidth, int columnHeight) {
    behaviorImageDisplayLabel_ = new QLabel(this);
    behaviorImageDisplayLabel_->setFixedSize(previewWidth, columnHeight);
    QVBoxLayout *behaviorColumnLayout = new QVBoxLayout();
    behaviorColumnLayout->addWidget(new QLabel("Behavior preview", this));
    behaviorColumnLayout->addWidget(behaviorImageDisplayLabel_);
    behaviorColumnLayout->addStretch();
    // Add timer for behavior display updates
    imageDisplayTimer_ = new QTimer(this);
    connect(
        imageDisplayTimer_,
        &QTimer::timeout,
        this,
        &MainGUIWindow::updateBehaviorImageDisplay);
    imageDisplayTimer_->start(1000 / streamingBehaviorFPS_);
    return behaviorColumnLayout;
}

QLayout *MainGUIWindow::createMusclePreviewColumn(
    int previewWidth, int previewHeight) {
    muscleImageDisplayLabel_ = new QLabel(this);
    muscleImageDisplayLabel_->setFixedSize(previewWidth, previewHeight);

    // Histogram + normalization-range slider for the muscle preview, shown only
    // while muscle imaging is enabled (toggled by the checkbox above).
    int histogramDisplayMin = recorderConfig_.getParameter<int>(
        "muscle_camera", "histogram_display_min");
    int histogramDisplayMax = recorderConfig_.getParameter<int>(
        "muscle_camera", "histogram_display_max");
    int defaultDisplayVmin = recorderConfig_.getParameter<int>(
        "muscle_camera", "default_display_vmin");
    int defaultDisplayVmax = recorderConfig_.getParameter<int>(
        "muscle_camera", "default_display_vmax");
    muscleHistogramWidget_ = new MuscleHistogramWidget(
        histogramDisplayMin,
        histogramDisplayMax,
        defaultDisplayVmin,
        defaultDisplayVmax,
        this);
    muscleHistogramWidget_->setFixedWidth(previewWidth);
    muscleHistogramWidget_->setVisible(false);

    // Muscle-imaging on/off checkbox, right-aligned in the column title so it
    // sits at the right edge of the muscle preview.
    muscleImagingCheckBox_ = new QCheckBox("Enable", this);
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
            // The histogram/range slider is only meaningful with a live muscle
            // preview.
            muscleHistogramWidget_->setVisible(enabled);
            // Re-stream so the controller switches modes immediately (sending a
            // STREAM mid-recording would revert the controller and abort it).
            if (!programState_->isRecording.load()) {
                arduinoCommunication_->stream(buildStreamingParams());
            }
        });

    // Stack the muscle preview directly on top of its histogram (no gap between
    // the two), then place that stack under the column title (title text on the
    // left, the Enable checkbox right-aligned to the preview's right edge).
    QVBoxLayout *musclePreviewStack = new QVBoxLayout();
    musclePreviewStack->setSpacing(0);
    musclePreviewStack->setContentsMargins(0, 0, 0, 0);
    musclePreviewStack->addWidget(muscleImageDisplayLabel_);
    musclePreviewStack->addWidget(muscleHistogramWidget_);
    QHBoxLayout *muscleTitleLayout = new QHBoxLayout();
    muscleTitleLayout->setContentsMargins(0, 0, 0, 0);
    muscleTitleLayout->addWidget(new QLabel("Muscle preview", this));
    muscleTitleLayout->addStretch();
    muscleTitleLayout->addWidget(muscleImagingCheckBox_);
    QVBoxLayout *muscleColumnLayout = new QVBoxLayout();
    muscleColumnLayout->addLayout(muscleTitleLayout);
    muscleColumnLayout->addLayout(musclePreviewStack);
    muscleColumnLayout->addStretch();
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
    return muscleColumnLayout;
}

QLayout *MainGUIWindow::createStagePreviewColumn() {
    motionControlWidget_ = new MotionControlWidget(
        recorderConfig_,
        trackingControlState_,
        stageMinXMm_,
        stageMaxXMm_,
        stageMinYMm_,
        stageMaxYMm_,
        this);
    // Tracking on/off checkbox, right-aligned in the column title so it sits at
    // the right edge of the stage preview.
    trackingEnabledCheckBox_ = new QCheckBox("Tracking", this);
    trackingEnabledCheckBox_->setChecked(true);
    connect(
        trackingEnabledCheckBox_,
        &QCheckBox::checkStateChanged,
        this,
        [this](int state) {
            if (state == Qt::Checked) {
                trackingControlState_->trackingOn.store(true);
            } else {
                trackingControlState_->trackingOn.store(false);
            }
        });
    QHBoxLayout *stageTitleLayout = new QHBoxLayout();
    stageTitleLayout->setContentsMargins(0, 0, 0, 0);
    stageTitleLayout->addWidget(new QLabel("Stage position", this));
    stageTitleLayout->addStretch();
    stageTitleLayout->addWidget(trackingEnabledCheckBox_);
    QVBoxLayout *stageColumnLayout = new QVBoxLayout();
    stageColumnLayout->addLayout(stageTitleLayout);
    stageColumnLayout->addWidget(motionControlWidget_);
    stageColumnLayout->addStretch();
    return stageColumnLayout;
}

QLayout *MainGUIWindow::createLiveImageDisplays() {
    // Each preview sits in its own column under a title; the columns are
    // separated by explicit spacers and top-aligned via a trailing stretch so
    // their titles and tops line up.
    QHBoxLayout *liveImageDisplayLayout = new QHBoxLayout();
    liveImageDisplayLayout->setSpacing(0);

    int muscleCameraPreviewWidth =
        recorderConfig_.getParameter<int>("gui", "muscle_camera_preview_width");
    int muscleCameraPreviewHeight =
        recorderConfig_.getParameter<int>("gui", "muscle_camera_preview_height");
    // The muscle column is the muscle preview stacked on top of its histogram /
    // slider; size the behavior preview to that combined height so the two
    // columns line up.
    int muscleColumnHeight = muscleCameraPreviewHeight + kHistogramWidgetHeight;

    // Derive the behavior preview width from the displayed behavior frame's
    // aspect ratio, so the image fills the label exactly with no left/right
    // padding (which would otherwise widen the gap to the muscle preview beyond
    // the muscle-to-stage gap). The behavior frame is rotated 90 degrees for
    // display (see reorientBehaviorImage), so its displayed width:height ratio is
    // the ROI height:width.
    int behaviorROIWidth =
        recorderConfig_.getParameter<int>("behavior_camera", "roi_width");
    int behaviorROIHeight =
        recorderConfig_.getParameter<int>("behavior_camera", "roi_height");
    int behaviorCameraPreviewWidth = calculateBehaviorCameraPreviewWidth(
        muscleColumnHeight, behaviorROIHeight, behaviorROIWidth);

    liveImageDisplayLayout->addLayout(createBehaviorPreviewColumn(
        behaviorCameraPreviewWidth, muscleColumnHeight));
    // 0.5x the muscle-to-stage gap separates the behavior and muscle columns.
    liveImageDisplayLayout->addSpacing(20);
    liveImageDisplayLayout->addLayout(createMusclePreviewColumn(
        muscleCameraPreviewWidth, muscleCameraPreviewHeight));
    // Motion stage state display, placed to the right of the muscle preview to
    // keep the window from getting too tall. It stays there whether or not
    // muscle imaging is enabled. Same 20 px gap between the muscle and stage
    // columns.
    liveImageDisplayLayout->addSpacing(20);
    liveImageDisplayLayout->addLayout(createStagePreviewColumn());
    liveImageDisplayLayout->addStretch();
    return liveImageDisplayLayout;
}

QLayout *MainGUIWindow::createRecordStopButtons() {
    // Slightly larger than the other buttons and placed to the right of the
    // config text boxes (see final layout assembly).
    recordButton_ = new QPushButton("Record", this);
    stopButton_ = new QPushButton("Stop", this);
    stopButton_->setEnabled(false); // initially disabled
    recordButton_->setMinimumSize(120, 50);
    stopButton_->setMinimumSize(120, 50);
    QVBoxLayout *recordStopButtonsLayout = new QVBoxLayout();
    recordStopButtonsLayout->addStretch();
    recordStopButtonsLayout->addWidget(recordButton_);
    recordStopButtonsLayout->addWidget(stopButton_);
    recordStopButtonsLayout->addStretch();
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
    return recordStopButtonsLayout;
}

void MainGUIWindow::setupProgrammedStopTimer() {
    // The acquirer threads stop recording exactly on the programmed frame count
    // by themselves; this only finalizes the GUI (reverts the UI and cameras to
    // streaming) shortly after.
    QTimer *programmedStopCheckTimer = new QTimer(this);
    connect(
        programmedStopCheckTimer,
        &QTimer::timeout,
        this,
        [this]() {
            if (programmedRecordingStop_->programmedStopReached.load()) {
                spdlog::info("Protocol stop reached. Finalizing recording.");
                endRecording(/*reachedProgrammedEnd=*/true);
                programmedRecordingStop_->numBehaviorFramesExpected = -1;
                programmedRecordingStop_->numMuscleFramesExpected = -1;
                programmedRecordingStop_->programmedStopReached.store(
                    false); // toggle off
                QMessageBox::information(
                    this,
                    "Recording stopped",
                    "End of protocol reached. Recording stopped.");
            }
        });
    programmedStopCheckTimer->start(500); // Check every 0.5 second
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

    // Warn if the save directory already exists and is non-empty, letting the
    // user overwrite it, auto-increment to a free directory, or cancel.
    if (!confirmOrResolveSaveDirectory()) {
        return;
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

    // Save the recording metadata into the freshly created save directory. These
    // read the recording parameters directly off the widgets.
    writeExperimentParameters(muscleNominalExposureUs, muscleBufferTimeUs);
    writeRecorderConfig();
    writeBehaviorCalibrationParameters();

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

    spdlog::info(
        "Recording STARTED: {} recording, muscle imaging {}. Saving to '{}'",
        currentRecordingIsScheduled_ ? "scheduled" : "open",
        muscleImagingCheckBox_->isChecked() ? "enabled" : "disabled",
        saveDirectory_->getDirectory().string());
}

bool MainGUIWindow::confirmOrResolveSaveDirectory() {
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
            return false;
        }
    }
    return true;
}

void MainGUIWindow::writeExperimentParameters(
    int muscleNominalExposureUs, int muscleBufferTimeUs) {
    std::filesystem::path outputPath =
        saveDirectory_->getDirectory() / "metadata/experiment_parameters.yaml";
    bool muscleImagingEnabled = muscleImagingCheckBox_->isChecked();

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "behavior_fps" << YAML::Value
        << behaviorFPSSpinBox_->value();
    out << YAML::Key << "muscle_imaging_enabled" << YAML::Value
        << muscleImagingEnabled;
    out << YAML::Key << "muscle_sync_ratio" << YAML::Value
        << syncRatioSpinBox_->value();
    // Convert ms to us.
    out << YAML::Key << "behavior_exposure_time_us" << YAML::Value
        << static_cast<int>(behaviorExposureTimeSpinBox_->value() * 1000);
    out << YAML::Key << "muscle_light_on_time_us" << YAML::Value
        << static_cast<int>(muscleLightOnTimeSpinBox_->value() * 1000);
    // The derived continuous-mode (auto-sequence) timing -- the nominal per-line
    // exposure programmed into the camera and the slack in the common-time window
    // beyond the light-on time -- is only meaningful when imaging muscle.
    if (muscleImagingEnabled) {
        out << YAML::Key << "muscle_nominal_exposure_us" << YAML::Value
            << muscleNominalExposureUs;
        out << YAML::Key << "muscle_buffer_time_us" << YAML::Value
            << muscleBufferTimeUs;
    }
    out << YAML::Key << "experiment_protocol" << YAML::Value
        << experimentProtocol_->toPlainText().toStdString();
    out << YAML::EndMap;

    std::ofstream fout(outputPath);
    if (!fout.is_open()) {
        std::string errorMessage =
            "Failed to open file for writing experiment parameters: " +
            outputPath.string();
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }
    fout << out.c_str();
    fout.close();

    spdlog::info("Saved experiment parameters to '{}'", outputPath.string());
}

void MainGUIWindow::writeRecorderConfig() {
    std::filesystem::path outputPath =
        saveDirectory_->getDirectory() / "metadata/recorder_config.yaml";
    recorderConfig_.saveToFile(outputPath);
    spdlog::info("Saved recorder config to '{}'", outputPath.string());
}

void MainGUIWindow::writeBehaviorCalibrationParameters() {
    std::filesystem::path outputPath =
        saveDirectory_->getDirectory() /
        "metadata/calibration_parameters_behavior.yaml";
    behaviorCamCalibrationParams_.saveToFile(outputPath);
    spdlog::info(
        "Saved behavior calibration parameters to '{}'", outputPath.string());
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

    spdlog::info(
        "Recording STOPPED ({}): {} recording, muscle imaging {}",
        reachedProgrammedEnd ? "reached scheduled end" : "user-initiated stop",
        currentRecordingIsScheduled_ ? "scheduled" : "open",
        muscleImagingCheckBox_->isChecked() ? "enabled" : "disabled");

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
    // Increment at least once, then keep incrementing until we land on a free
    // (non-existent or empty) directory, matching the auto-increment behavior
    // used when starting a recording (see confirmOrResolveSaveDirectory).
    std::filesystem::path incremented =
        incrementDirectoryName(saveDirectory_->getDirectory().string());
    while (std::filesystem::is_directory(incremented) &&
           !std::filesystem::is_empty(incremented)) {
        incremented = incrementDirectoryName(incremented.string());
    }
    directoryLineEdit_->setText(QString::fromStdString(incremented.string()));
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
    // Update the live histogram from the raw 16-bit frame, then normalize the
    // preview using the [vmin, vmax] window selected on the slider.
    muscleHistogramWidget_->setImage(latestFrame);
    cv::Mat processedFrame;
    convert16BitTo8Bit(
        latestFrame,
        processedFrame,
        muscleHistogramWidget_->vmin(),
        muscleHistogramWidget_->vmax());
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
 * programmed steps. If any steps are given, the protocol must contain exactly
 * one "<n>/x/stop" step and it must be the very last step. Returns the number
 * of steps, or -1 on a malformed string.
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
            "If any steps are given, the protocol must contain exactly one "
            "'x/stop' step, and it must be the very last step.\n\n"
            "Do not add a trailing ';' at the end.\n\n"
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

    // If any steps are given, require exactly one stop step, at the very end.
    if (numStepsParsed > 0) {
        int numStops = 0;
        for (const OperationStep &step : opSequence) {
            if (step.op == OpType::STOP) {
                ++numStops;
            }
        }
        if (numStops != 1 || opSequence.back().op != OpType::STOP) {
            return reportError();
        }
    }

    return numStepsParsed;
}
