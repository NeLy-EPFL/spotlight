#pragma once

#include <array>
#include <filesystem>
#include <thread>
#include <tuple>

#include <QApplication>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <functional>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "recorder/common/behavior_recording.h"
#include "recorder/common/cli.h"
#include "recorder/common/data_types.h"
#include "recorder/common/gui_widgets.h"
#include "recorder/common/muscle_recording.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/arduino_communication.h"
#include "recorder/peripherals/behavior_camera.h"
#include "recorder/peripherals/muscle_camera.h"

// QLabel that reports left clicks, in the coordinates of its pixmap (shown
// unscaled at the top left)
class ClickableImageLabel : public QLabel {
  public:
    std::function<void(int x, int y)> on_click;

  protected:
    void mousePressEvent(QMouseEvent *event) override;
};

// Live view of the behavior camera and the two muscle cameras. Clicking a
// muscle image selects its ROI center; each muscle image has a histogram with
// a range slider that sets its display intensity range. "Save ROIs" saves both
// ROIs and closes the window; "Cancel" closes it.
class AlignCamerasWindow : public QWidget {
  public:
    AlignCamerasWindow(
        const RecorderConfig &recorder_config,
        std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
        std::shared_ptr<MuscleRecordingState> muscle_recording_state,
        QWidget *parent = nullptr);

  private:
    void update_images();

    std::shared_ptr<BehaviorRecordingState> behavior_recording_state_;
    // Latest frame of each muscle camera (calcium, fiducial)
    std::array<std::shared_ptr<LatestFrame>, 2> muscle_frame_holders_;
    QTimer timer_;
    QLabel *behavior_label_;
    std::array<ClickableImageLabel *, 2> muscle_labels_;
    std::array<MuscleHistogramWidget *, 2> histogram_widgets_;
};

void align_camera(const std::filesystem::path &profile_dir);
