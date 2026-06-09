#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <queue>
#include <tuple>
#include <vector>

#include <QCheckBox>
#include <QCloseEvent>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QSpinBox>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "recorder/common/behavior_recording.h"
#include "recorder/common/calibration.h"
#include "recorder/common/muscle_recording.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/tracking_control.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/arduino_communication.h"

#include <comm_protocol/protocol.h>

// Forward declaration from main.hpp
bool quitProgram();

class MotionControlWidget : public QWidget {
  public:
    MotionControlWidget(
        const RecorderConfig &recorderConfig,
        std::shared_ptr<TrackingControlState> trackingControlState,
        double minXAbsoluteMm,
        double maxXAbsoluteMm,
        double minYAbsoluteMm,
        double maxYAbsoluteMm,
        QWidget *parent = nullptr);
    ~MotionControlWidget();

  protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

  private:
    int mapToPixelX(float x) const;
    int mapToPixelY(float y) const;
    float mapToStageX(int x) const;
    float mapToStageY(int y) const;

    QTimer timer_;
    float minXAbsoluteMm_;
    float maxXAbsoluteMm_;
    float minYAbsoluteMm_;
    float maxYAbsoluteMm_;

    std::shared_ptr<TrackingControlState> trackingControlState_;
};

// Live histogram of the muscle camera image with a two-handle range slider
// underneath. The two handles select the [vmin, vmax] intensity window used to
// normalize the displayed muscle image (pixels <= vmin are black, >= vmax are
// white). The min handle can never cross past the max handle. Both the
// histogram x-axis and the slider span the fixed [histogramMin, histogramMax]
// intensity range read from the recorder config.
class MuscleHistogramWidget : public QWidget {
  public:
    MuscleHistogramWidget(
        int histogramMin,
        int histogramMax,
        int defaultVmin,
        int defaultVmax,
        QWidget *parent = nullptr);

    // Recompute the histogram from a 16-bit (CV_16UC1) muscle frame and repaint.
    void setImage(const cv::Mat &image16Bit);

    int vmin() const { return vmin_; }
    int vmax() const { return vmax_; }

  protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

  private:
    int valueToX(int value) const;
    int xToValue(int x) const;

    enum class DraggedHandle { None, Min, Max };

    int histogramMin_;
    int histogramMax_;
    int vmin_;
    int vmax_;
    std::vector<float> histogram_; // bin heights normalized to [0, 1]
    DraggedHandle draggedHandle_ = DraggedHandle::None;
};

class MainGUIWindow : public QWidget {
    Q_OBJECT

  public:
    explicit MainGUIWindow(
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
        QWidget *parent = nullptr);

  private slots:
    void startRecording();
    void stopRecording();
    void updateBehaviorImageDisplay();
    void updateMuscleImageDisplay();
    void browseDirectory();
    void incrementDirectory();

  private:
    // Assemble the STREAM / START_RECORDING params from the current widget
    // values and the cached PCO timing.
    TriggerParams buildStreamingParams() const;
    TriggerParams buildRecordingParams() const;
    // Program the free-running (auto-sequence) muscle camera's nominal exposure
    // so it produces muscle frames at behFrameRate / syncRatio. Called at
    // startup, when a recording starts (recording rate), and when it ends
    // (streaming rate).
    void pushMuscleCameraExposure(int behFrameRate, int syncRatio);
    // Shared by the Stop button (manual) and the programmed-stop timer.
    // reachedProgrammedEnd is true when a scheduled recording ran to its end
    // (the controller has already reverted on its own).
    void endRecording(bool reachedProgrammedEnd);
    // Pre-flight for startRecording(): parse the experiment protocol, record the
    // programmed-stop frame counts, and (when imaging muscle) derive + validate
    // the continuous-mode muscle timing. Returns false (after showing an error
    // dialog) if the protocol string or the muscle timing is invalid; on success
    // fills opSequence and the derived muscle timing (both 0 when muscle imaging
    // is off).
    bool validateAndPrepareRecording(
        std::deque<OperationStep> &opSequence,
        int &muscleNominalExposureUs,
        int &muscleBufferTimeUs);

    std::shared_ptr<ProgramState> programState_;
    QSpinBox *behaviorFPSSpinBox_;
    QSpinBox *syncRatioSpinBox_;
    QDoubleSpinBox *behaviorExposureTimeSpinBox_;
    QDoubleSpinBox *muscleLightOnTimeSpinBox_;
    QTextEdit *experimentProtocol_;
    QLineEdit *directoryLineEdit_;
    QCheckBox *trackingEnabledCheckBox_;
    QCheckBox *muscleImagingCheckBox_;
    MotionControlWidget *motionControlWidget_;
    QPushButton *recordButton_;
    QPushButton *stopButton_;
    QLabel *behaviorImageDisplayLabel_;
    QLabel *muscleImageDisplayLabel_;
    MuscleHistogramWidget *muscleHistogramWidget_;
    QTimer *imageDisplayTimer_;
    RecorderConfig recorderConfig_;
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState_;
    std::shared_ptr<MuscleRecordingState> muscleRecordingState_;
    std::shared_ptr<TrackingControlState> trackingControlState_;
    CalibrationParams &behaviorCamCalibrationParams_;
    ActiveAreaMask &activeAreaMask_;
    std::shared_ptr<SaveDirectory> saveDirectory_;
    std::shared_ptr<ArduinoCommunication> arduinoCommunication_;
    std::shared_ptr<ProgrammedStop> programmedRecordingStop_;
    double stageMinXMm_;
    double stageMaxXMm_;
    double stageMinYMm_;
    double stageMaxYMm_;

    int streamingBehaviorFPS_ = 0;
    int streamingSyncRatio_ = 1;
    // Default behavior exposure / muscle light-on times (us), from the recorder
    // config. Used for the streaming params the controller always runs; the
    // spin-box values are buffered for recording only (see buildStreamingParams).
    int defaultBehExpTimeUs_ = 0;
    int defaultMuscLightOnTimeUs_ = 0;
    bool muscleImagingEnabled_ = false;

    // PCO sensor timing sent to the controller so it can derive the muscle
    // trigger delay (rolling time = scanned lines * line time).
    unsigned int pcoCamRollingTimeUs_ = 0;
    unsigned int pcoCamReadoutTimeUs_ = 0;
    // True while the in-progress recording is a scheduled one (non-empty
    // opSequence). Controls how the recording is ended (see endRecording()).
    bool currentRecordingIsScheduled_ = false;

  protected:
    void closeEvent(QCloseEvent *event) override;
};

// Helpers
cv::Mat addCornerMarker(
    const cv::Mat &image,
    double arenaSizeXMm,
    double arenaSizeYMm,
    MotionStagePosition stagePosition,
    const CalibrationParams &behaviorCamCalibrationParams);

int parseProtocolString(
    const std::string &protocolTextFieldString,
    std::deque<OperationStep> &opSequence);

std::string incrementDirectoryName(const std::string &path);
