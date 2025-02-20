#ifndef GUI_HPP
#define GUI_HPP

#include <memory>
#include <atomic>

#include <QWidget>
#include <QPushButton>
#include <QSerialPort>
#include <QSerialPortInfo>

#include "utils.hpp"

class Gui : public QWidget
{
    Q_OBJECT

public:
    explicit Gui(std::shared_ptr<std::atomic<bool>> isSavingData,
                 std::shared_ptr<std::atomic<bool>> toQuit,
                 QWidget *parent = nullptr);

private slots:
    void startRecording();
    void stopRecording();

private:
    std::shared_ptr<std::atomic<bool>> isSavingData;
    std::shared_ptr<std::atomic<bool>> toQuit;
    std::string directory;
    std::string serialPortName;
    QSerialPort serialPort;
    QPushButton *recordButton;
    QPushButton *stopButton;
};

#endif // GUI_HPP
