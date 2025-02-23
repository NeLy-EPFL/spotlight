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

#include "utils.hpp"
#include "constants.hpp"
#include "global.hpp"
#include "recordingController.hpp"
#include "peripherals/triggering.hpp"

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
    void browseDirectory();

private:
    std::string serialPortName_;
    QSerialPort serialPort_;
    QSpinBox *behaviorFPSSpinBox_;
    QDoubleSpinBox *behaviorExposureTimeSpinBox_;
    QLineEdit *directoryLineEdit_;
    QPushButton *recordButton_;
    QPushButton *stopButton_;
    QLabel *behaviorImageDisplayLabel_;
    QTimer *imageDisplayTimer_;

protected:
    void closeEvent(QCloseEvent *event) override;
};

#endif // GUI_HPP
