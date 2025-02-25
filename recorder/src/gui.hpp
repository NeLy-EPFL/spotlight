#ifndef GUI_HPP
#define GUI_HPP

#include <memory>
#include <atomic>
#include <queue>
#include <mutex>

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
#include "constants.hpp"
#include "global.hpp"
#include "recordingController.hpp"
#include "peripherals/triggering.hpp"

class MotionControlWidget : public QWidget
{
public:
    MotionControlWidget(QWidget *parent = nullptr);
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
    float minXAbsoluteMm_ = MOTION_STAGE_X_MIN_PHYSICAL_MM;
    float maxXAbsoluteMm_ = MOTION_STAGE_X_MAX_PHYSICAL_MM;
    float minYAbsoluteMm_ = MOTION_STAGE_Y_MIN_PHYSICAL_MM;
    float maxYAbsoluteMm_ = MOTION_STAGE_Y_MAX_PHYSICAL_MM;
};

class MainGUIWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainGUIWindow(QWidget *parent = nullptr);

private slots:
    void startRecording();
    void stopRecording();
    void updateImageDisplay();
    void browseDirectory();

private:
    std::string serialPortName_;
    QSerialPort serialPort_;
    QSpinBox *behaviorFPSSpinBox_;
    QDoubleSpinBox *behaviorExposureTimeSpinBox_;
    QLineEdit *directoryLineEdit_;
    MotionControlWidget *motionControlWidget_;
    QPushButton *recordButton_;
    QPushButton *stopButton_;
    QLabel *behaviorImageDisplayLabel_;
    QTimer *imageDisplayTimer_;

protected:
    void closeEvent(QCloseEvent *event) override;
};

#endif // GUI_HPP
