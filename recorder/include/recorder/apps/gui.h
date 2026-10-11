#pragma once

#include <atomic>
#include <deque>
#include <filesystem>
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
#include "recorder/common/gui_widgets.h"
#include "recorder/common/muscle_recording.h"
#include "recorder/common/recorder.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/tracking_control.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/arduino_communication.h"

#include <comm_protocol/protocol.h>

class MotionControlWidget : public QWidget {
  public:
    MotionControlWidget(
        const RecorderConfig &recorder_config,
        std::shared_ptr<TrackingControlState> tracking_control_state,
        double min_x_absolute_mm,
        double max_x_absolute_mm,
        double min_y_absolute_mm,
        double max_y_absolute_mm,
        QWidget *parent = nullptr);
    ~MotionControlWidget();

  protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

  private:
    int map_to_pixel_x(float x) const;
    int map_to_pixel_y(float y) const;
    float map_to_stage_x(int x) const;
    float map_to_stage_y(int y) const;

    QTimer timer_;
    float min_x_absolute_mm_;
    float max_x_absolute_mm_;
    float min_y_absolute_mm_;
    float max_y_absolute_mm_;

    std::shared_ptr<TrackingControlState> tracking_control_state_;
};

// The main window of run-spotlight: a front end around Recorder, which holds
// the recording logic. The window reads the recording parameters off its
// widgets, resolves the save directory with the user, and shows the previews.
class MainGUIWindow : public QWidget {
    Q_OBJECT

  public:
    explicit MainGUIWindow(Recorder &recorder, QWidget *parent = nullptr);
    // PROFILING ONLY: step 1 enables muscle imaging, 2 starts recording, 3
    // stops it (driven by SIGUSR1, see run_spotlight_main)
    void profiling_step(int step);

  private slots:
    void start_recording();
    void stop_recording();
    void update_behavior_image_display();
    void update_muscle_image_display();
    void browse_directory();
    void increment_directory();

  private:
    // Enable the controls that apply to the current mode (recording or
    // streaming): the record/stop buttons and the muscle-imaging checkbox,
    // which cannot be toggled during a recording or without muscle cameras.
    void update_controls(bool recording);
    // The recording parameters currently entered in the widgets
    RecordingParams get_recording_params() const;

    // --- Constructor helpers ---
    // Each create*Row()/create*Column() builds one part of the GUI, stores the
    // relevant widget pointers in members, and returns the assembled layout.
    QLayout *create_behavior_fps_row();
    QLayout *create_sync_ratio_row();
    QLayout *create_behavior_exposure_row();
    QLayout *create_muscle_exposure_row();
    QLayout *create_protocol_row();
    QLayout *create_save_directory_row(int config_rows_width, int buttons_gap);
    QLayout *create_behavior_preview_pane(int preview_width, int column_height);
    QLayout *create_muscle_preview_pane(int preview_width, int preview_height);
    QLayout *create_stage_preview_pane();
    QLayout *create_live_image_displays();
    QLayout *create_record_stop_buttons();
    // Poll for a programmed (protocol-driven) stop and finalize the GUI when it
    // is reached.
    void setup_programmed_stop_timer();

    // Resolve a save directory that already exists and is non-empty by
    // prompting the user (overwrite / auto-increment / cancel). Returns false
    // if the user cancelled, in which case the recording must not start.
    bool confirm_or_resolve_save_directory();

    Recorder &recorder_;
    const RecorderConfig &recorder_config_;
    QSpinBox *behavior_fps_spin_box_;
    QSpinBox *sync_ratio_spin_box_;
    QDoubleSpinBox *behavior_exposure_time_spin_box_;
    QDoubleSpinBox *muscle_light_on_time_spin_box_;
    QTextEdit *experiment_protocol_;
    QLineEdit *directory_line_edit_;
    QCheckBox *tracking_enabled_check_box_;
    QCheckBox *muscle_imaging_check_box_;
    MotionControlWidget *motion_control_widget_;
    QPushButton *record_button_;
    QPushButton *stop_button_;
    QLabel *behavior_image_display_label_;
    QLabel *muscle_image_display_label_;
    // Channels of the muscle preview: calcium (green), fiducials (red), and
    // behavior (blue). Only the calcium and fiducial channels have a histogram
    // (the behavior channel is displayed like the behavior preview).
    QCheckBox *calcium_channel_check_box_;
    QCheckBox *fiducial_channel_check_box_;
    QCheckBox *behavior_channel_check_box_;
    MuscleHistogramWidget *calcium_histogram_widget_;
    MuscleHistogramWidget *fiducial_histogram_widget_;
    QTimer *image_display_timer_;

  protected:
    void closeEvent(QCloseEvent *event) override;
};

// Helpers
// Draw the arena outline onto a copy of a (reoriented) behavior image.
// view_scale: size of the image relative to the camera image (< 1 for a
// downscaled preview).
cv::Mat add_corner_marker(
    const cv::Mat &image,
    double arena_size_x_mm,
    double arena_size_y_mm,
    MotionStagePosition stage_position,
    const CalibrationParams &behavior_cam_calibration_params,
    double view_scale = 1.0);

// Map a displayed (reoriented) camera image into the displayed calcium camera
// view, of size `calcium_view_size`. Placeholder for the live registration:
// for now the image centers are aligned without scaling (i.e. the image is
// cropped or zero-padded).
cv::Mat
warp_to_calcium_view(const cv::Mat &image, const cv::Size &calcium_view_size);

std::string increment_directory_name(const std::string &path);
