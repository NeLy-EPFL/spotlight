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

// Defined in run_spotlight_main.cc
bool quit_program();

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

// Live histogram of the muscle camera image with a two-handle range slider
// underneath. The two handles select the [vmin, vmax] intensity window used to
// normalize the displayed muscle image (pixels <= vmin are black, >= vmax are
// white). The min handle can never cross past the max handle. Both the
// histogram x-axis and the slider span the fixed [histogram_min, histogram_max]
// intensity range read from the recorder config.
class MuscleHistogramWidget : public QWidget {
  public:
    MuscleHistogramWidget(
        int histogram_min,
        int histogram_max,
        int default_vmin,
        int default_vmax,
        QWidget *parent = nullptr);

    // Recompute the histogram from a 16-bit (CV_16UC1) muscle frame and
    // repaint.
    void set_image(const cv::Mat &image16_bit);

    int vmin() const {
        return vmin_;
    }
    int vmax() const {
        return vmax_;
    }

  protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

  private:
    int value_to_x(int value) const;
    int x_to_value(int x) const;

    enum class DraggedHandle { none, min, max };

    int histogram_min_;
    int histogram_max_;
    int vmin_;
    int vmax_;
    std::vector<float> histogram_; // bin heights normalized to [0, 1]
    DraggedHandle dragged_handle_ = DraggedHandle::none;
};

class MainGUIWindow : public QWidget {
    Q_OBJECT

  public:
    explicit MainGUIWindow(
        const RecorderConfig &recorder_config,
        std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
        std::shared_ptr<MuscleRecordingState> muscle_recording_state,
        std::shared_ptr<TrackingControlState> tracking_control_state,
        CalibrationParams &behavior_cam_calibration_params,
        std::shared_ptr<SaveDirectory> save_directory,
        std::shared_ptr<ArduinoCommunication> arduino_communication,
        std::shared_ptr<ProgramState> program_state,
        std::shared_ptr<ProgrammedStop> programmed_recording_stop,
        ActiveAreaMask &active_area_mask,
        double stage_min_x_mm,
        double stage_max_x_mm,
        double stage_min_y_mm,
        double stage_max_y_mm,
        QWidget *parent = nullptr);

  private slots:
    void start_recording();
    void stop_recording();
    void update_behavior_image_display();
    void update_muscle_image_display();
    void browse_directory();
    void increment_directory();

  private:
    // Assemble the STREAM / START_RECORDING params from the current widget
    // values and the cached PCO timing.
    TriggerParams build_streaming_params() const;
    TriggerParams build_recording_params() const;
    // Program the free-running (auto-sequence) muscle camera's nominal exposure
    // so it produces muscle frames at beh_frame_rate / sync_ratio. Called at
    // startup, when a recording starts (recording rate), and when it ends
    // (streaming rate).
    void push_muscle_camera_exposure(int beh_frame_rate, int sync_ratio);
    // Shared by the Stop button (manual) and the programmed-stop timer.
    // reached_programmed_end is true when a scheduled recording ran to its end
    // (the controller has already reverted on its own).
    void end_recording(bool reached_programmed_end);
    // Pre-flight for start_recording(): parse the experiment protocol, record
    // the programmed-stop frame counts, and (when imaging muscle) derive +
    // validate the continuous-mode muscle timing. Returns false (after showing
    // an error dialog) if the protocol string or the muscle timing is invalid;
    // on success fills op_sequence and the derived muscle timing (both 0 when
    // muscle imaging is off).
    bool validate_and_prepare_recording(
        std::deque<OperationStep> &op_sequence,
        int &muscle_nominal_exposure_us,
        int &muscle_buffer_time_us);

    // --- Constructor helpers ---
    // Read the streaming parameters and cache the PCO sensor timing into
    // members (must run before the create*() helpers and
    // build_streaming_params).
    void load_recording_parameters();
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

    // --- start_recording() helpers ---
    // Resolve a save directory that already exists and is non-empty by
    // prompting the user (overwrite / auto-increment / cancel). Returns false
    // if the user cancelled, in which case the recording must not start.
    bool confirm_or_resolve_save_directory();
    // Write the recording metadata files into the (already created) save
    // directory, reading the recording parameters directly from the widgets.
    // The muscle timing passed to write_experiment_parameters is the derived
    // continuous-mode timing from validate_and_prepare_recording().
    void write_experiment_parameters(
        int muscle_nominal_exposure_us, int muscle_buffer_time_us);
    void write_recorder_config();
    void write_behavior_calibration_parameters();

    std::shared_ptr<ProgramState> program_state_;
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
    MuscleHistogramWidget *muscle_histogram_widget_;
    QTimer *image_display_timer_;
    RecorderConfig recorder_config_;
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state_;
    std::shared_ptr<MuscleRecordingState> muscle_recording_state_;
    std::shared_ptr<TrackingControlState> tracking_control_state_;
    CalibrationParams &behavior_cam_calibration_params_;
    ActiveAreaMask &active_area_mask_;
    std::shared_ptr<SaveDirectory> save_directory_;
    std::shared_ptr<ArduinoCommunication> arduino_communication_;
    std::shared_ptr<ProgrammedStop> programmed_recording_stop_;
    double stage_min_x_mm_;
    double stage_max_x_mm_;
    double stage_min_y_mm_;
    double stage_max_y_mm_;

    int streaming_behavior_fps_ = 0;
    int streaming_sync_ratio_ = 1;
    // Default behavior exposure / muscle light-on times (us), from the recorder
    // config. Used for the streaming params the controller always runs; the
    // spin-box values are buffered for recording only (see
    // build_streaming_params).
    int default_beh_exp_time_us_ = 0;
    int default_musc_light_on_time_us_ = 0;
    bool muscle_imaging_enabled_ = false;

    // PCO sensor timing sent to the controller so it can derive the muscle
    // trigger delay (rolling time = scanned lines * line time).
    unsigned int pco_cam_rolling_time_us_ = 0;
    unsigned int pco_cam_readout_time_us_ = 0;
    // True while the in-progress recording is a scheduled one (non-empty
    // op_sequence). Controls how the recording is ended (see end_recording()).
    bool current_recording_is_scheduled_ = false;

  protected:
    void closeEvent(QCloseEvent *event) override;
};

// Helpers
cv::Mat add_corner_marker(
    const cv::Mat &image,
    double arena_size_x_mm,
    double arena_size_y_mm,
    MotionStagePosition stage_position,
    const CalibrationParams &behavior_cam_calibration_params);

int parse_protocol_string(
    const std::string &protocol_text_field_string,
    std::deque<OperationStep> &op_sequence);

std::string increment_directory_name(const std::string &path);
