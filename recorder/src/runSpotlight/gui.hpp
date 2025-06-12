#ifndef GUI_HPP
#define GUI_HPP

#include <memory>
#include <atomic>
#include <queue>
#include <mutex>
#include <tuple>
#include <vector>

#include <QWidget>
#include <QPushButton>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QCheckBox>
#include <QTextEdit>
#include <QLineEdit>
#include <QLabel>
#include <QTimer>
#include <QFileDialog>
#include <QCloseEvent>
#include <QMessageBox>
#include <QPainter>

#include "../common/utils.hpp"
#include "../common/recorderConfig.hpp"
#include "../common/behaviorRecording.hpp"
#include "../common/muscleRecording.hpp"
#include "../common/trackingControl.hpp"
#include "../common/calibration.hpp"
#include "../peripherals/arduinoCommunication.hpp"
#include "../peripherals/experimentProtocol.hpp"

// Forward declaration from main.hpp
bool quitProgram();

class MotionControlWidget : public QWidget
{
public:
    MotionControlWidget(
        const RecorderConfig &recorderConfig,
        std::shared_ptr<TrackingControlState> trackingControlState,
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

class MainGUIWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainGUIWindow(
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
        QWidget *parent = nullptr);

private slots:
    void startRecording();
    void stopRecording();
    void updateBehaviorImageDisplay();
    void updateMuscleImageDisplay();
    void browseDirectory();

private:
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
    QTimer *imageDisplayTimer_;
    RecorderConfig recorderConfig_;
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState_;
    std::shared_ptr<MuscleRecordingState> muscleRecordingState_;
    std::shared_ptr<TrackingControlState> trackingControlState_;
    CalibrationParams &behaviorCamCalibrationParams_;
    CalibrationParams &muscleCamCalibrationParams_;
    std::shared_ptr<SaveDirectory> saveDirectory_;
    std::shared_ptr<ArduinoCommunication> arduinoCommunication_;
    std::shared_ptr<ProgrammedStop> programmedRecordingStop_;
    std::shared_ptr<DualRecordingConfig> dualRecordingConfigForSaving_;
    DualRecordingConfig *dualRecordingConfigForStreaming_;

    int muscleImage16To8BitScale_ = 1;
    int muscleImage16To8BitOffset_ = 0;
    int streamingBehaviorFPS_ = 0;
    int streamingSyncRatio_ = INT_MAX;
    bool muscleImagingEnabled_ = false;

protected:
    void closeEvent(QCloseEvent *event) override;
};

class DualRecordingConfigWindow : public QDialog
{
    Q_OBJECT
public:
    explicit DualRecordingConfigWindow(
        const RecorderConfig &recorderConfig,
        std::shared_ptr<DualRecordingConfig> dualRecordingConfig,
        const MuscleCameraROI &muscleCameraROI,
        QWidget *parent = nullptr);
    ~DualRecordingConfigWindow();

private slots:
    void onButtonClicked();

private:
    QVBoxLayout *mainLayout_;
    QSpinBox *behaviorCameraFPSLineEdit_;
    QSpinBox *syncRatioLineEdit_;
    QDoubleSpinBox *muscleLightOnTimeLineEdit_;
    QPushButton *withMuscleButton_;
    QPushButton *withoutMuscleButton_;

    int desiredWidth_ = 400;
    int desiredHeight_ = 350;

    const RecorderConfig &recorderConfig_;
    const MuscleCameraROI &muscleCameraROI_;
    std::shared_ptr<DualRecordingConfig> dualRecordingConfigForSaving_;
};

// Helpers
cv::Mat addCornerMarker(cv::Mat image,
                        int arenaSizeXmm,
                        int arenaSizeYmm,
                        MotionStagePosition stagePosition,
                        CalibrationParams &behaviorCamCalibrationParams);

int parseProtocolString(const std::string &protocolTextFieldString,
                        std::vector<ProtocolStep> &steps);
#endif // GUI_HPP
