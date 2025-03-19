#ifndef GUI_HPP
#define GUI_HPP

#include <memory>
#include <atomic>
#include <queue>
#include <mutex>
#include <tuple>

#include <QWidget>
#include <QPushButton>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QTimer>
#include <QFileDialog>
#include <QCloseEvent>
#include <QMessageBox>
#include <QPainter>

#include "utils.hpp"
#include "recorderConfig.hpp"
#include "behaviorRecording.hpp"
#include "trackingControl.hpp"
#include "calibration.hpp"
#include "peripherals/triggering.hpp"

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
        std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
        std::shared_ptr<TrackingControlState> trackingControlState,
        CalibrationParams &behaviorCamCalibrationParams,
        std::shared_ptr<SaveDirectory> saveDirectory,
        std::shared_ptr<LatestFrame> latestBehaviorFrameHolder,
        std::shared_ptr<ArduinoTriggerInterface> arduinoTriggerInterface,
        QWidget *parent = nullptr);

private slots:
    void startRecording();
    void stopRecording();
    void updateImageDisplay();
    void browseDirectory();

private:
    QSpinBox *behaviorFPSSpinBox_;
    QDoubleSpinBox *behaviorExposureTimeSpinBox_;
    QLineEdit *directoryLineEdit_;
    MotionControlWidget *motionControlWidget_;
    QPushButton *recordButton_;
    QPushButton *stopButton_;
    QLabel *behaviorImageDisplayLabel_;
    QTimer *imageDisplayTimer_;
    RecorderConfig recorderConfig_;
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState_;
    std::shared_ptr<TrackingControlState> trackingControlState_;
    CalibrationParams &behaviorCamCalibrationParams_;
    std::shared_ptr<SaveDirectory> saveDirectory_;
    std::shared_ptr<LatestFrame> latestBehaviorFrameHolder_;
    std::shared_ptr<ArduinoTriggerInterface> arduinoTriggerInterface_;

protected:
    void closeEvent(QCloseEvent *event) override;
};

// Helpers
cv::Mat addCornerMarker(cv::Mat image,
                        MotionStagePosition stagePosition,
                        CalibrationParams &behaviorCamCalibrationParams);

#endif // GUI_HPP
