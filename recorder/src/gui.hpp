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
#include <QLabel>
#include <QTimer>

#include "utils.hpp"
#include "constants.hpp"
#include "global.hpp"

cv::Mat getLatestFrame();
QImage cvMatToQImage(const cv::Mat &mat);

class Gui : public QWidget
{
    Q_OBJECT

public:
    explicit Gui(QWidget *parent = nullptr);

private slots:
    void startRecording();
    void stopRecording();
    void updateImageDisplay();

private:
    std::shared_ptr<std::atomic<bool>> toQuit;
    std::string directory;
    std::string serialPortName;
    QSerialPort serialPort;
    QPushButton *recordButton;
    QPushButton *stopButton;
    QSpinBox *behaviorFPSSpinBox;
    QDoubleSpinBox *behaviorExposureTimeSpinBox;
    QLabel *behaviorImageDisplayLabel;
    QTimer *imageDisplayTimer;
};

#endif // GUI_HPP
