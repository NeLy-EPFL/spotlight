#ifndef GUI_HPP
#define GUI_HPP

#include <atomic>
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
#include <QPainter>
#include <QPushButton>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QSpinBox>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "../common/behaviorRecording.hpp"
#include "../common/calibration.hpp"
#include "../common/muscleRecording.hpp"
#include "../common/recorderConfig.hpp"
#include "../common/trackingControl.hpp"
#include "../common/utils.hpp"
#include "../peripherals/arduinoCommunication.hpp"
#include "../peripherals/experimentProtocol.hpp"

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

class MainGUIWindow : public QWidget {
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
    ActiveAreaMask &activeAreaMask_;
    std::shared_ptr<SaveDirectory> saveDirectory_;
    std::shared_ptr<ArduinoCommunication> arduinoCommunication_;
    std::shared_ptr<ProgrammedStop> programmedRecordingStop_;
    std::shared_ptr<DualRecordingConfig> dualRecordingConfigForSaving_;
    std::unique_ptr<DualRecordingConfig> dualRecordingConfigForStreaming_;
    double stageMinXMm_;
    double stageMaxXMm_;
    double stageMinYMm_;
    double stageMaxYMm_;

    int muscleImage16To8BitScale_ = 1;
    int muscleImage16To8BitOffset_ = 0;
    int streamingBehaviorFPS_ = 0;
    int streamingSyncRatio_ = INT_MAX;
    bool muscleImagingEnabled_ = false;

  protected:
    void closeEvent(QCloseEvent *event) override;
};

class DualRecordingConfigWindow : public QDialog {
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
    QSpinBox *behaviorCameraFPSSpinBox_;
    QSpinBox *syncRatioSpinBox_;
    QDoubleSpinBox *muscleLightOnTimeSpinBox_;
    QPushButton *withMuscleButton_;
    QPushButton *withoutMuscleButton_;

    int desiredWidth_ = 400;
    int desiredHeight_ = 350;

    const RecorderConfig &recorderConfig_;
    const MuscleCameraROI &muscleCameraROI_;
    std::shared_ptr<DualRecordingConfig> dualRecordingConfigForSaving_;
};

// Helpers
cv::Mat addCornerMarker(
    cv::Mat image,
    double arenaSizeXMm,
    double arenaSizeYMm,
    MotionStagePosition stagePosition,
    CalibrationParams &behaviorCamCalibrationParams);

int parseProtocolString(
    const std::string &protocolTextFieldString,
    std::vector<ProtocolStep> &steps);

std::string incrementDirectoryName(const std::string &path);
#endif // GUI_HPP
