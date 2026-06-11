#include "recorder/apps/gui.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace {
// Layout constants for the muscle histogram + range slider widget.
constexpr int histogram_num_bins = 256;
constexpr int histogram_widget_height = 65;
constexpr int slider_area_height = 12;
constexpr int handle_half_width = 5;

QImage cv_mat_to_q_image(const cv::Mat &mat) {
    if (mat.empty()) {
        return QImage();
    }
    if (mat.channels() == 1) {
        return QImage(
            mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
    }
    // 3-channel BGR -> RGB for Qt
    cv::Mat rgb;
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888)
        .copy();
}
} // namespace

std::string increment_directory_name(const std::string &path) {
    size_t end = path.size();
    while (end > 0 && path[end - 1] == '/') {
        --end;
    }
    size_t start = end;
    while (start > 0 &&
           std::isdigit(static_cast<unsigned char>(path[start - 1]))) {
        --start;
    }

    if (start == end)
        return path.substr(0, end) + "_001/";

    std::string num_str = path.substr(start, end - start);
    int num = std::stoi(num_str) + 1;
    int width = static_cast<int>(num_str.size());

    std::ostringstream oss;
    oss << path.substr(0, start) << std::setfill('0') << std::setw(width) << num
        << "/";
    return oss.str();
}

MuscleHistogramWidget::MuscleHistogramWidget(
    int histogram_min,
    int histogram_max,
    int default_vmin,
    int default_vmax,
    QWidget *parent)
    : QWidget(parent), histogram_min_(histogram_min),
      histogram_max_(histogram_max), vmin_(default_vmin), vmax_(default_vmax),
      histogram_(histogram_num_bins, 0.0f) {
    setMinimumHeight(histogram_widget_height);
}

void MuscleHistogramWidget::set_image(const cv::Mat &image16_bit) {
    if (image16_bit.empty()) {
        return;
    }
    int num_bins = static_cast<int>(histogram_.size());
    int channels[] = {0};
    int hist_size[] = {num_bins};
    // calcHist's upper bound is exclusive, so add 1 to include histogram_max_.
    float value_range[] = {
        static_cast<float>(histogram_min_),
        static_cast<float>(histogram_max_ + 1)};
    const float *ranges[] = {value_range};
    cv::Mat hist;
    cv::calcHist(
        &image16_bit, 1, channels, cv::Mat(), hist, 1, hist_size, ranges);
    // Normalize bin heights to the tallest bin so the histogram fills the
    // available height regardless of frame size / brightness.
    double max_bin = 0.0;
    cv::minMaxLoc(hist, nullptr, &max_bin);
    for (int i = 0; i < num_bins; ++i) {
        histogram_[i] = max_bin > 0.0
                            ? hist.at<float>(i) / static_cast<float>(max_bin)
                            : 0.0f;
    }
    update();
}

int MuscleHistogramWidget::value_to_x(int value) const {
    int usable_width = std::max(1, width() - 2 * handle_half_width);
    return handle_half_width + (value - histogram_min_) * usable_width /
                                   std::max(1, histogram_max_ - histogram_min_);
}

int MuscleHistogramWidget::x_to_value(int x) const {
    int usable_width = std::max(1, width() - 2 * handle_half_width);
    int value = histogram_min_ + (x - handle_half_width) *
                                     (histogram_max_ - histogram_min_) /
                                     usable_width;
    return std::clamp(value, histogram_min_, histogram_max_);
}

void MuscleHistogramWidget::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);

    int slider_top = height() - slider_area_height;
    int histogram_height = slider_top;

    painter.fillRect(rect(), QColor(30, 30, 30));

    // Histogram bars.
    int num_bins = static_cast<int>(histogram_.size());
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(180, 180, 180));
    for (int i = 0; i < num_bins; ++i) {
        int x0 = width() * i / num_bins;
        int x1 = width() * (i + 1) / num_bins;
        int bar_height = static_cast<int>(histogram_[i] * histogram_height);
        painter.drawRect(
            x0,
            histogram_height - bar_height,
            std::max(1, x1 - x0),
            bar_height);
    }

    int x_min = value_to_x(vmin_);
    int x_max = value_to_x(vmax_);

    // Dim the regions outside the selected [vmin, vmax] window.
    painter.setBrush(QColor(0, 0, 0, 130));
    painter.drawRect(0, 0, x_min, histogram_height);
    painter.drawRect(x_max, 0, width() - x_max, histogram_height);

    // Slider groove and selected span.
    int groove_y = slider_top + slider_area_height / 2;
    painter.setPen(QPen(QColor(120, 120, 120), 2));
    painter.drawLine(
        handle_half_width, groove_y, width() - handle_half_width, groove_y);
    painter.setPen(QPen(QColor(80, 160, 240), 3));
    painter.drawLine(x_min, groove_y, x_max, groove_y);

    // Min handle (blue) with a guide line over the histogram.
    painter.setPen(QPen(QColor(80, 160, 240), 1));
    painter.drawLine(x_min, 0, x_min, histogram_height);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(80, 160, 240));
    painter.drawRect(
        x_min - handle_half_width,
        slider_top,
        2 * handle_half_width,
        slider_area_height);

    // Max handle (orange) with a guide line over the histogram.
    painter.setPen(QPen(QColor(240, 160, 60), 1));
    painter.drawLine(x_max, 0, x_max, histogram_height);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(240, 160, 60));
    painter.drawRect(
        x_max - handle_half_width,
        slider_top,
        2 * handle_half_width,
        slider_area_height);

    // Value labels.
    painter.setPen(Qt::white);
    painter.drawText(
        QRect(2, 0, width() - 4, 14),
        Qt::AlignLeft,
        QString("min %1").arg(vmin_));
    painter.drawText(
        QRect(2, 0, width() - 4, 14),
        Qt::AlignRight,
        QString("max %1").arg(vmax_));
}

void MuscleHistogramWidget::mousePressEvent(QMouseEvent *event) {
    int x = static_cast<int>(event->position().x());
    // Grab whichever handle is closer to the click.
    dragged_handle_ =
        std::abs(x - value_to_x(vmin_)) <= std::abs(x - value_to_x(vmax_))
            ? DraggedHandle::min
            : DraggedHandle::max;
    mouseMoveEvent(event);
}

void MuscleHistogramWidget::mouseMoveEvent(QMouseEvent *event) {
    if (dragged_handle_ == DraggedHandle::none) {
        return;
    }
    int value = x_to_value(static_cast<int>(event->position().x()));
    if (dragged_handle_ == DraggedHandle::min) {
        // The min handle can never move past the max handle.
        vmin_ = std::min(value, vmax_);
    } else {
        vmax_ = std::max(value, vmin_);
    }
    update();
}

MotionControlWidget::MotionControlWidget(
    const RecorderConfig &recorder_config,
    std::shared_ptr<TrackingControlState> tracking_control_state,
    double min_x_absolute_mm,
    double max_x_absolute_mm,
    double min_y_absolute_mm,
    double max_y_absolute_mm,
    QWidget *parent)
    : QWidget(parent), tracking_control_state_(std::move(tracking_control_state)) {
    // Stage bounds are derived externally (in run_spotlight_main) from the
    // arena dimensions and the fitted calibration model, then passed in here.
    min_x_absolute_mm_ = min_x_absolute_mm;
    max_x_absolute_mm_ = max_x_absolute_mm;
    min_y_absolute_mm_ = min_y_absolute_mm;
    max_y_absolute_mm_ = max_y_absolute_mm;

    int gui_motion_stage_preview_update_freq =
        recorder_config.get_parameter<int>(
            "gui", "motion_stage_preview_update_frequency_hz");
    int gui_motion_stage_preview_height = recorder_config.get_parameter<int>(
        "gui", "motion_stage_preview_height");
    // Enlarge the stage preview by 50% over its configured height; the width is
    // derived from the height below, so it scales by the same factor.
    gui_motion_stage_preview_height = gui_motion_stage_preview_height * 3 / 2;

    connect(
        &timer_,
        &QTimer::timeout,
        this,
        QOverload<>::of(&MotionControlWidget::update));
    timer_.start(1000 / gui_motion_stage_preview_update_freq); // in ms
    int gui_motion_stage_preview_width =
        calculate_behavior_camera_preview_width(
            gui_motion_stage_preview_height,
            max_x_absolute_mm_ - min_x_absolute_mm_,
            max_y_absolute_mm_ - min_y_absolute_mm_);
    setFixedSize(
        gui_motion_stage_preview_width, gui_motion_stage_preview_height);
}

MotionControlWidget::~MotionControlWidget() {
    timer_.stop();
}

void MotionControlWidget::paintEvent(QPaintEvent *event) {
    if (!tracking_control_state_->motion_control_handler_ready.load()) {
        return;
    }

    Q_UNUSED(event);
    QPainter painter(this);

    // Draw the light gray rectangle representing the stage boundaries
    painter.fillRect(rect(), QColor(220, 220, 220));
    painter.setPen(Qt::black);
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    // Calculate where to draw the red dot representing the stage position
    MotionStagePosition curr_stage_position;
    {
        std::lock_guard<std::mutex> lock(
            tracking_control_state_->latest_motion_stage_position_mutex);
        curr_stage_position =
            tracking_control_state_->latest_motion_stage_position;
    }
    const float physical_x = curr_stage_position.x_pos_mm;
    const float physical_y = curr_stage_position.y_pos_mm;
    int pixel_x = map_to_pixel_x(physical_x);
    int pixel_y = map_to_pixel_y(physical_y);

    // Draw the red dot
    painter.setPen(Qt::red);
    painter.setBrush(Qt::red);
    int dot_diameter = 10;
    painter.drawEllipse(
        pixel_x - dot_diameter / 2,
        pixel_y - dot_diameter / 2,
        dot_diameter,
        dot_diameter);

    // Draw coordinate labels
    painter.setPen(Qt::black);
    int min_field_width = 0;
    int precision = 2;
    int text_position_x = 10;
    int text_position_y = 20;
    painter.drawText(
        text_position_x,
        text_position_y,
        QString("(%1, %2) mm")
            .arg(physical_x, min_field_width, 'f', precision)
            .arg(physical_y, min_field_width, 'f', precision));
}

void MotionControlWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        float stage_x = map_to_stage_x(event->position().x());
        float stage_y = map_to_stage_y(event->position().y());
        spdlog::debug("Clicked at ({}, {})", stage_x, stage_y);
        tracking_control_state_->overriding_pos_x.store(stage_x);
        tracking_control_state_->overriding_pos_y.store(stage_y);
        tracking_control_state_->should_override_tracking.store(true);
    }
}

int MotionControlWidget::map_to_pixel_x(float x) const {
    return (x - min_x_absolute_mm_) /
               (max_x_absolute_mm_ - min_x_absolute_mm_) * width() +
           0.5;
}

int MotionControlWidget::map_to_pixel_y(float y) const {
    return (y - min_y_absolute_mm_) /
               (max_y_absolute_mm_ - min_y_absolute_mm_) * height() +
           0.5;
}

float MotionControlWidget::map_to_stage_x(int x) const {
    return x / static_cast<float>(width()) *
               (max_x_absolute_mm_ - min_x_absolute_mm_) +
           min_x_absolute_mm_;
}

float MotionControlWidget::map_to_stage_y(int y) const {
    return y / static_cast<float>(height()) *
               (max_y_absolute_mm_ - min_y_absolute_mm_) +
           min_y_absolute_mm_;
}

MainGUIWindow::MainGUIWindow(
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
    QWidget *parent)
    : QWidget(parent), recorder_config_(recorder_config),
      behavior_recording_state_(std::move(behavior_recording_state)),
      muscle_recording_state_(std::move(muscle_recording_state)),
      tracking_control_state_(std::move(tracking_control_state)),
      behavior_cam_calibration_params_(behavior_cam_calibration_params),
      save_directory_(std::move(save_directory)),
      arduino_communication_(std::move(arduino_communication)),
      program_state_(std::move(program_state)),
      programmed_recording_stop_(std::move(programmed_recording_stop)),
      active_area_mask_(active_area_mask), stage_min_x_mm_(stage_min_x_mm),
      stage_max_x_mm_(stage_max_x_mm), stage_min_y_mm_(stage_min_y_mm),
      stage_max_y_mm_(stage_max_y_mm) {
    load_recording_parameters();

    // Arrange layout. The config text boxes (and their labels) occupy a
    // fixed-width column on the left, wide enough that the labels and spin
    // boxes are not cramped. The record/stop buttons sit immediately to the
    // right of that column with a small gap. The save directory row joins the
    // same column below the protocol box, so the vertical gap above it matches
    // the spacing between the config rows.
    const int config_rows_width = 700;
    const int buttons_gap = 12;

    QVBoxLayout *config_rows_layout = new QVBoxLayout();
    config_rows_layout->setContentsMargins(0, 0, 0, 0);
    config_rows_layout->addLayout(create_behavior_fps_row());
    config_rows_layout->addLayout(create_sync_ratio_row());
    config_rows_layout->addLayout(create_behavior_exposure_row());
    config_rows_layout->addLayout(create_muscle_exposure_row());
    config_rows_layout->addLayout(create_protocol_row());
    QWidget *config_rows_panel = new QWidget(this);
    config_rows_panel->setLayout(config_rows_layout);
    config_rows_panel->setFixedWidth(config_rows_width);

    // Config rows on the left, the buttons immediately to their right (with a
    // gap), and a trailing stretch so the buttons stay next to the boxes rather
    // than being pushed to the far edge.
    QHBoxLayout *config_and_buttons_layout = new QHBoxLayout();
    config_and_buttons_layout->setContentsMargins(0, 0, 0, 0);
    config_and_buttons_layout->addWidget(config_rows_panel);
    config_and_buttons_layout->addSpacing(buttons_gap);
    config_and_buttons_layout->addLayout(create_record_stop_buttons());
    config_and_buttons_layout->addStretch();

    // The config-rows-plus-buttons block and the save directory row share one
    // column so the save directory is separated from the protocol box by the
    // same vertical spacing as the config rows are from each other. Both rows
    // start at x = 0 and use the same config_rows_width + buttons_gap, so the
    // save directory text box aligns with the spin boxes above and the Browse /
    // Increment buttons align with the Record / Stop buttons. The panel is left
    // unconstrained in width so the (horizontally laid out) Browse / Increment
    // buttons are never clipped; the trailing stretch in each row absorbs the
    // slack.
    QVBoxLayout *top_block_layout = new QVBoxLayout();
    top_block_layout->setContentsMargins(0, 0, 0, 0);
    top_block_layout->addLayout(config_and_buttons_layout);
    top_block_layout->addLayout(
        create_save_directory_row(config_rows_width, buttons_gap));
    QWidget *top_block_panel = new QWidget(this);
    top_block_panel->setLayout(top_block_layout);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(top_block_panel, 0, Qt::AlignLeft);
    layout->addLayout(create_live_image_displays());
    setLayout(layout);

    setup_programmed_stop_timer();

    // Start in streaming mode: live preview only, not saving. Muscle imaging is
    // off by default (enable_muscle = false: behavior camera free-runs, blue
    // excitation LED off). The controller is configured with a single STREAM
    // command.
    record_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    muscle_imaging_check_box_->setEnabled(true);
    program_state_->is_recording.store(false);
    arduino_communication_->stream(build_streaming_params());
}

void MainGUIWindow::load_recording_parameters() {
    streaming_behavior_fps_ = recorder_config_.get_parameter<int>(
        "behavior_camera", "streaming_frame_rate");

    // Load rolling shutter parameter
    double rolling_shutter_line_time_us =
        recorder_config_.get_parameter<double>(
            "muscle_camera", "rolling_shutter_line_time_us");
    int muscle_cam_readout_time_us = recorder_config_.get_parameter<double>(
        "muscle_camera", "sensor_readout_time_us");

    // Load streaming sync ratio
    streaming_sync_ratio_ = recorder_config_.get_parameter<int>(
        "muscle_camera", "streaming_sync_ratio");

    // The muscle camera is started (and waited for) in run_spotlight_main
    // before this window is constructed, so it is ready by now.

    // Cache the PCO sensor timing sent in every STREAM / START_RECORDING. The
    // controller derives the muscle trigger delay from these (the rolling time
    // is the time to scan all lines of the muscle ROI).
    pco_cam_rolling_time_us_ = static_cast<unsigned int>(
        muscle_recording_state_->muscle_camera->get_num_lines_scanned() *
        rolling_shutter_line_time_us);
    pco_cam_readout_time_us_ =
        static_cast<unsigned int>(muscle_cam_readout_time_us);
}

QLayout *MainGUIWindow::create_behavior_fps_row() {
    behavior_fps_spin_box_ = new QSpinBox(this);
    behavior_fps_spin_box_->setRange(1, 1000);
    int behavior_camera_default_recording_frame_rate =
        recorder_config_.get_parameter<int>(
            "behavior_camera", "default_recording_fps");
    behavior_fps_spin_box_->setValue(
        behavior_camera_default_recording_frame_rate);
    // Don't connect to ArduinoCommunication! This value is only used during
    // recording. When streaming, the behavior frame rate comes from
    // behavior_camera/streaming_frame_rate and this field is ignored.
    QHBoxLayout *behavior_fps_layout = new QHBoxLayout();
    behavior_fps_layout->addWidget(new QLabel("Behavior FPS (Hz)"));
    behavior_fps_layout->addWidget(behavior_fps_spin_box_);
    return behavior_fps_layout;
}

QLayout *MainGUIWindow::create_sync_ratio_row() {
    sync_ratio_spin_box_ = new QSpinBox(this);
    sync_ratio_spin_box_->setRange(1, INT_MAX);
    int sync_ratio = recorder_config_.get_parameter<int>(
        "muscle_camera", "default_recording_sync_ratio");
    sync_ratio_spin_box_->setValue(sync_ratio);
    // Muscle-only parameter: disabled unless muscle imaging is enabled (the
    // checkbox below toggles it). Don't connect to ArduinoCommunication! This
    // value is only used during recording. When streaming, the sync ratio comes
    // from muscle_camera/streaming_sync_ratio and this field is ignored.
    sync_ratio_spin_box_->setEnabled(false);
    QHBoxLayout *sync_ratio_layout = new QHBoxLayout();
    sync_ratio_layout->addWidget(new QLabel("Behavior FPS : muscle FPS"));
    sync_ratio_layout->addWidget(sync_ratio_spin_box_);
    return sync_ratio_layout;
}

QLayout *MainGUIWindow::create_behavior_exposure_row() {
    behavior_exposure_time_spin_box_ = new QDoubleSpinBox(this);
    behavior_exposure_time_spin_box_->setRange(0.001, 1000.0);
    default_beh_exp_time_us_ = recorder_config_.get_parameter<int>(
        "behavior_camera", "default_exposure_time_us");
    behavior_exposure_time_spin_box_->setValue(
        default_beh_exp_time_us_ / 1000.0);
    // Buffered recording parameter: applied only when a recording starts (see
    // build_recording_params). During streaming the controller always runs the
    // default behavior exposure (see build_streaming_params), so editing this
    // spin box has no live effect and is intentionally not re-streamed -- the
    // controller's status display must show the actual live behavior, not the
    // not-yet-executed recording config.
    QHBoxLayout *behavior_exposure_time_layout = new QHBoxLayout();
    behavior_exposure_time_layout->addWidget(
        new QLabel("Behavior exposure time (ms)"));
    behavior_exposure_time_layout->addWidget(behavior_exposure_time_spin_box_);
    return behavior_exposure_time_layout;
}

QLayout *MainGUIWindow::create_muscle_exposure_row() {
    muscle_light_on_time_spin_box_ = new QDoubleSpinBox(this);
    muscle_light_on_time_spin_box_->setRange(0.001, 1000.0);
    default_musc_light_on_time_us_ = recorder_config_.get_parameter<int>(
        "muscle_camera", "default_light_on_time_us");
    muscle_light_on_time_spin_box_->setValue(
        default_musc_light_on_time_us_ / 1000.0);
    // Initialize the free-running (auto-sequence) muscle camera to the
    // streaming muscle frame rate. In continuous mode the nominal exposure sets
    // the frame rate; it is switched to the recording rate when a recording
    // starts and back when it ends (see start_recording/end_recording).
    push_muscle_camera_exposure(streaming_behavior_fps_, streaming_sync_ratio_);
    // Muscle-only parameter: disabled unless muscle imaging is enabled (the
    // checkbox below toggles it). Like the behavior exposure, this is a
    // buffered recording parameter -- applied only when a recording starts;
    // during streaming the controller always runs the default light-on time, so
    // editing it has no live effect and is not re-streamed.
    muscle_light_on_time_spin_box_->setEnabled(false);
    QHBoxLayout *muscle_light_on_time_layout = new QHBoxLayout();
    muscle_light_on_time_layout->addWidget(
        new QLabel("Muscle exposure (light-on) time (ms)"));
    muscle_light_on_time_layout->addWidget(muscle_light_on_time_spin_box_);
    return muscle_light_on_time_layout;
}

QLayout *MainGUIWindow::create_protocol_row() {
    QLabel *protocol_label = new QLabel("Experiment protocol", this);
    experiment_protocol_ = new QTextEdit(this);
    experiment_protocol_->setMinimumHeight(40);
    QVBoxLayout *protocol_layout = new QVBoxLayout();
    protocol_layout->addWidget(protocol_label);
    protocol_layout->addWidget(experiment_protocol_);
    return protocol_layout;
}

QLayout *MainGUIWindow::create_save_directory_row(
    int config_rows_width, int buttons_gap) {
    directory_line_edit_ = new QLineEdit(this);
    directory_line_edit_->setText(save_directory_->get_directory().c_str());
    connect(
        directory_line_edit_,
        &QLineEdit::textChanged,
        this,
        [this](const QString &text) {
            spdlog::debug("saveDirectory changed to {}", text.toStdString());
            save_directory_->set_directory(text.toStdString());
        });
    QPushButton *browse_button = new QPushButton("Browse", this);
    QPushButton *increment_button = new QPushButton("Increment", this);

    // The label + text box occupy the same fixed-width column as the config
    // rows, so the text box's right edge aligns with the spin boxes above. The
    // Browse / Increment buttons then sit (after the same gap) left-aligned to
    // the Record / Stop buttons column.
    QHBoxLayout *label_and_edit_layout = new QHBoxLayout();
    label_and_edit_layout->setContentsMargins(0, 0, 0, 0);
    label_and_edit_layout->addWidget(new QLabel("Save directory"));
    label_and_edit_layout->addWidget(directory_line_edit_);
    QWidget *label_and_edit_panel = new QWidget(this);
    label_and_edit_panel->setLayout(label_and_edit_layout);
    label_and_edit_panel->setFixedWidth(config_rows_width);

    QHBoxLayout *directory_layout = new QHBoxLayout();
    directory_layout->setContentsMargins(0, 0, 0, 0);
    directory_layout->addWidget(label_and_edit_panel);
    directory_layout->addSpacing(buttons_gap);
    directory_layout->addWidget(browse_button);
    directory_layout->addWidget(increment_button);
    directory_layout->addStretch();

    connect(
        browse_button,
        &QPushButton::clicked,
        this,
        &MainGUIWindow::browse_directory);
    connect(
        increment_button,
        &QPushButton::clicked,
        this,
        &MainGUIWindow::increment_directory);
    return directory_layout;
}

QLayout *MainGUIWindow::create_behavior_preview_pane(
    int preview_width, int column_height) {
    behavior_image_display_label_ = new QLabel(this);
    behavior_image_display_label_->setFixedSize(preview_width, column_height);
    QVBoxLayout *behavior_column_layout = new QVBoxLayout();
    behavior_column_layout->addWidget(new QLabel("Behavior preview", this));
    behavior_column_layout->addWidget(behavior_image_display_label_);
    behavior_column_layout->addStretch();
    // Add timer for behavior display updates
    image_display_timer_ = new QTimer(this);
    connect(
        image_display_timer_,
        &QTimer::timeout,
        this,
        &MainGUIWindow::update_behavior_image_display);
    image_display_timer_->start(1000 / streaming_behavior_fps_);
    return behavior_column_layout;
}

QLayout *MainGUIWindow::create_muscle_preview_pane(
    int preview_width, int preview_height) {
    muscle_image_display_label_ = new QLabel(this);
    muscle_image_display_label_->setFixedSize(preview_width, preview_height);

    // Histogram + normalization-range slider for the muscle preview, shown only
    // while muscle imaging is enabled (toggled by the checkbox above).
    int histogram_display_min = recorder_config_.get_parameter<int>(
        "muscle_camera", "histogram_display_min");
    int histogram_display_max = recorder_config_.get_parameter<int>(
        "muscle_camera", "histogram_display_max");
    int default_display_vmin = recorder_config_.get_parameter<int>(
        "muscle_camera", "default_display_vmin");
    int default_display_vmax = recorder_config_.get_parameter<int>(
        "muscle_camera", "default_display_vmax");
    muscle_histogram_widget_ = new MuscleHistogramWidget(
        histogram_display_min,
        histogram_display_max,
        default_display_vmin,
        default_display_vmax,
        this);
    muscle_histogram_widget_->setFixedWidth(preview_width);
    muscle_histogram_widget_->setVisible(false);

    // Muscle-imaging on/off checkbox, right-aligned in the column title so it
    // sits at the right edge of the muscle preview.
    muscle_imaging_check_box_ = new QCheckBox("Enable", this);
    muscle_imaging_check_box_->setChecked(false);
    connect(
        muscle_imaging_check_box_,
        &QCheckBox::checkStateChanged,
        this,
        [this](int state) {
            // Muscle imaging on/off is expressed by enable_muscle: when off the
            // controller free-runs the behavior camera with the blue excitation
            // LED disabled (build_streaming_params() reads
            // muscle_imaging_enabled_).
            bool enabled = (state == Qt::Checked);
            muscle_imaging_enabled_ = enabled;
            spdlog::info(
                enabled ? "Enabling muscle imaging"
                        : "Disabling muscle imaging");
            // The muscle-only parameters are editable only when imaging muscle.
            sync_ratio_spin_box_->setEnabled(enabled);
            muscle_light_on_time_spin_box_->setEnabled(enabled);
            // The histogram/range slider is only meaningful with a live muscle
            // preview.
            muscle_histogram_widget_->setVisible(enabled);
            // Re-stream so the controller switches modes immediately (sending a
            // STREAM mid-recording would revert the controller and abort it).
            if (!program_state_->is_recording.load()) {
                arduino_communication_->stream(build_streaming_params());
            }
        });

    // Stack the muscle preview directly on top of its histogram (no gap between
    // the two), then place that stack under the column title (title text on the
    // left, the Enable checkbox right-aligned to the preview's right edge).
    QVBoxLayout *muscle_preview_stack = new QVBoxLayout();
    muscle_preview_stack->setSpacing(0);
    muscle_preview_stack->setContentsMargins(0, 0, 0, 0);
    muscle_preview_stack->addWidget(muscle_image_display_label_);
    muscle_preview_stack->addWidget(muscle_histogram_widget_);
    QHBoxLayout *muscle_title_layout = new QHBoxLayout();
    muscle_title_layout->setContentsMargins(0, 0, 0, 0);
    muscle_title_layout->addWidget(new QLabel("Muscle preview", this));
    muscle_title_layout->addStretch();
    muscle_title_layout->addWidget(muscle_imaging_check_box_);
    QVBoxLayout *muscle_column_layout = new QVBoxLayout();
    muscle_column_layout->addLayout(muscle_title_layout);
    muscle_column_layout->addLayout(muscle_preview_stack);
    muscle_column_layout->addStretch();
    // Add timer for muscle display updates
    QTimer *muscle_image_display_timer = new QTimer(this);
    connect(
        muscle_image_display_timer,
        &QTimer::timeout,
        this,
        &MainGUIWindow::update_muscle_image_display);
    float muscle_streaming_fps =
        static_cast<float>(streaming_behavior_fps_) / streaming_sync_ratio_;
    spdlog::info("Muscle streaming FPS: {}", muscle_streaming_fps);
    muscle_image_display_timer->start(1000 / muscle_streaming_fps);
    return muscle_column_layout;
}

QLayout *MainGUIWindow::create_stage_preview_pane() {
    motion_control_widget_ = new MotionControlWidget(
        recorder_config_,
        tracking_control_state_,
        stage_min_x_mm_,
        stage_max_x_mm_,
        stage_min_y_mm_,
        stage_max_y_mm_,
        this);
    // Tracking on/off checkbox, right-aligned in the column title so it sits at
    // the right edge of the stage preview.
    tracking_enabled_check_box_ = new QCheckBox("Tracking", this);
    tracking_enabled_check_box_->setChecked(true);
    connect(
        tracking_enabled_check_box_,
        &QCheckBox::checkStateChanged,
        this,
        [this](int state) {
            if (state == Qt::Checked) {
                tracking_control_state_->tracking_on.store(true);
            } else {
                tracking_control_state_->tracking_on.store(false);
            }
        });
    QHBoxLayout *stage_title_layout = new QHBoxLayout();
    stage_title_layout->setContentsMargins(0, 0, 0, 0);
    stage_title_layout->addWidget(new QLabel("Stage position", this));
    stage_title_layout->addStretch();
    stage_title_layout->addWidget(tracking_enabled_check_box_);
    QVBoxLayout *stage_column_layout = new QVBoxLayout();
    stage_column_layout->addLayout(stage_title_layout);
    stage_column_layout->addWidget(motion_control_widget_);
    stage_column_layout->addStretch();
    return stage_column_layout;
}

QLayout *MainGUIWindow::create_live_image_displays() {
    // Each preview sits in its own column under a title; the columns are
    // separated by explicit spacers and top-aligned via a trailing stretch so
    // their titles and tops line up.
    QHBoxLayout *live_image_display_layout = new QHBoxLayout();
    live_image_display_layout->setSpacing(0);

    int muscle_camera_preview_width = recorder_config_.get_parameter<int>(
        "gui", "muscle_camera_preview_width");
    int muscle_camera_preview_height = recorder_config_.get_parameter<int>(
        "gui", "muscle_camera_preview_height");
    // The muscle column is the muscle preview stacked on top of its histogram /
    // slider; size the behavior preview to that combined height so the two
    // columns line up.
    int muscle_column_height =
        muscle_camera_preview_height + histogram_widget_height;

    // Derive the behavior preview width from the displayed behavior frame's
    // aspect ratio, so the image fills the label exactly with no left/right
    // padding (which would otherwise widen the gap to the muscle preview beyond
    // the muscle-to-stage gap). The behavior frame is rotated 90 degrees for
    // display (see reorientBehaviorImage), so its displayed width:height ratio
    // is the ROI height:width.
    int behavior_roi_width =
        recorder_config_.get_parameter<int>("behavior_camera", "roi_width");
    int behavior_roi_height =
        recorder_config_.get_parameter<int>("behavior_camera", "roi_height");
    int behavior_camera_preview_width = calculate_behavior_camera_preview_width(
        muscle_column_height, behavior_roi_height, behavior_roi_width);

    live_image_display_layout->addLayout(create_behavior_preview_pane(
        behavior_camera_preview_width, muscle_column_height));
    // 0.5x the muscle-to-stage gap separates the behavior and muscle columns.
    live_image_display_layout->addSpacing(20);
    live_image_display_layout->addLayout(create_muscle_preview_pane(
        muscle_camera_preview_width, muscle_camera_preview_height));
    // Motion stage state display, placed to the right of the muscle preview to
    // keep the window from getting too tall. It stays there whether or not
    // muscle imaging is enabled. Same 20 px gap between the muscle and stage
    // columns.
    live_image_display_layout->addSpacing(20);
    live_image_display_layout->addLayout(create_stage_preview_pane());
    live_image_display_layout->addStretch();
    return live_image_display_layout;
}

QLayout *MainGUIWindow::create_record_stop_buttons() {
    // Slightly larger than the other buttons and placed to the right of the
    // config text boxes (see final layout assembly).
    record_button_ = new QPushButton("Record", this);
    stop_button_ = new QPushButton("Stop", this);
    stop_button_->setEnabled(false); // initially disabled
    record_button_->setMinimumSize(120, 50);
    stop_button_->setMinimumSize(120, 50);
    QVBoxLayout *record_stop_buttons_layout = new QVBoxLayout();
    record_stop_buttons_layout->addStretch();
    record_stop_buttons_layout->addWidget(record_button_);
    record_stop_buttons_layout->addWidget(stop_button_);
    record_stop_buttons_layout->addStretch();
    connect(
        record_button_,
        &QPushButton::clicked,
        this,
        &MainGUIWindow::start_recording);
    connect(
        stop_button_,
        &QPushButton::clicked,
        this,
        &MainGUIWindow::stop_recording);
    return record_stop_buttons_layout;
}

void MainGUIWindow::setup_programmed_stop_timer() {
    // The acquirer threads stop recording exactly on the programmed frame count
    // by themselves; this only finalizes the GUI (reverts the UI and cameras to
    // streaming) shortly after.
    QTimer *programmed_stop_check_timer = new QTimer(this);
    connect(programmed_stop_check_timer, &QTimer::timeout, this, [this]() {
        if (programmed_recording_stop_->programmed_stop_reached.load()) {
            spdlog::info("Protocol stop reached. Finalizing recording.");
            end_recording(/*reachedProgrammedEnd=*/true);
            programmed_recording_stop_->num_behavior_frames_expected = -1;
            programmed_recording_stop_->num_muscle_frames_expected = -1;
            programmed_recording_stop_->programmed_stop_reached.store(
                false); // toggle off
            QMessageBox::information(
                this,
                "Recording stopped",
                "End of protocol reached. Recording stopped.");
        }
    });
    programmed_stop_check_timer->start(500); // Check every 0.5 second
}

bool MainGUIWindow::validate_and_prepare_recording(
    std::deque<OperationStep> &op_sequence,
    int &muscle_nominal_exposure_us,
    int &muscle_buffer_time_us) {
    muscle_nominal_exposure_us = 0;
    muscle_buffer_time_us = 0;

    // Parse the experiment protocol into an opSequence and record the
    // programmed-stop frame counts. A malformed string aborts the recording.
    int num_steps_parsed = parse_protocol_string(
        experiment_protocol_->toPlainText().toStdString(), op_sequence);
    spdlog::info("Parsed {} protocol steps", op_sequence.size());
    if (num_steps_parsed < 0) {
        // parseProtocolString has already shown a detailed error dialog.
        return false;
    } else if (num_steps_parsed == 0) {
        spdlog::info("GUI starting recording without any protocol steps");
        programmed_recording_stop_->num_behavior_frames_expected = -1;
        programmed_recording_stop_->num_muscle_frames_expected = -1;
    } else {
        spdlog::info(
            "GUI starting recording with {} protocol steps",
            op_sequence.size());
        programmed_recording_stop_->num_behavior_frames_expected =
            op_sequence.back().frame_idx;
        programmed_recording_stop_->num_muscle_frames_expected =
            op_sequence.back().frame_idx / sync_ratio_spin_box_->value();
        spdlog::info(
            "Setting expected number of steps to {} (behavior) and {} (muscle)",
            programmed_recording_stop_->num_behavior_frames_expected,
            programmed_recording_stop_->num_muscle_frames_expected);
    }
    // An empty opSequence is an open recording; a non-empty one is scheduled.
    current_recording_is_scheduled_ = num_steps_parsed > 0;

    // When imaging muscle, derive and validate the continuous-mode muscle
    // trigger timing from the current recording parameters.
    if (muscle_imaging_check_box_->isChecked()) {
        MuscleTriggerTiming muscle_trigger_timing(
            behavior_fps_spin_box_->value(),
            sync_ratio_spin_box_->value(),
            static_cast<int>(muscle_light_on_time_spin_box_->value() * 1000));
        double rolling_shutter_line_time_us =
            recorder_config_.get_parameter<double>(
                "muscle_camera", "rolling_shutter_line_time_us");
        int muscle_cam_readout_time_us = recorder_config_.get_parameter<double>(
            "muscle_camera", "sensor_readout_time_us");
        if (!muscle_trigger_timing.compute_parameters(
                muscle_recording_state_->muscle_camera->get_num_lines_scanned(),
                rolling_shutter_line_time_us,
                muscle_cam_readout_time_us)) {
            QMessageBox::critical(
                this,
                "Error",
                "Invalid muscle recording configuration. In particular, check "
                "that the muscle recording interval (1 / muscle FPS) is long "
                "enough for the rolling shutter, light-on, and sensor readout "
                "times. See "
                "https://github.com/NeLy-EPFL/spotlight-control/issues/79.");
            return false;
        }
        muscle_nominal_exposure_us =
            muscle_trigger_timing.get_nominal_exposure_us();
        muscle_buffer_time_us = muscle_trigger_timing.get_buffer_time_us();
    }

    return true;
}

void MainGUIWindow::start_recording() {
    // Check if behavior camera has been initialized
    if (!behavior_recording_state_->behavior_camera ||
        !behavior_recording_state_->behavior_camera->is_ready()) {
        spdlog::error("Behavior camera not ready. Cannot start recording.");
        // Make a pop-up error window
        QMessageBox::critical(
            this,
            "Error",
            "Behavior camera not ready yet. Please wait 10 seconds. If the "
            "error persists, something has gone wrong. Check logs for info.");
        return;
    }

    // Warn if the save directory already exists and is non-empty, letting the
    // user overwrite it, auto-increment to a free directory, or cancel.
    if (!confirm_or_resolve_save_directory()) {
        return;
    }

    // Validate the experiment protocol and (when imaging muscle) the muscle
    // trigger timing before any side effects -- camera exposure change, button
    // toggles, directory creation, metadata writes -- so an invalid
    // configuration aborts cleanly and leaves nothing behind. The derived
    // muscle timing is returned for the metadata and the exposure update below.
    std::deque<OperationStep> op_sequence;
    int muscle_nominal_exposure_us = 0;
    int muscle_buffer_time_us = 0;
    if (!validate_and_prepare_recording(
            op_sequence, muscle_nominal_exposure_us, muscle_buffer_time_us)) {
        return;
    }

    // Switch the free-running camera to the recording muscle frame rate before
    // START_RECORDING, so it is already emitting common-time onsets at the
    // recording cadence when the firmware begins locking the behavior frames to
    // them. The controller's camFlushTimeUs delay covers the transient while
    // the new exposure takes effect.
    if (muscle_imaging_check_box_->isChecked()) {
        muscle_recording_state_->muscle_camera->set_nominal_exposure_us(
            static_cast<unsigned int>(muscle_nominal_exposure_us));
    }

    // Toggle GUI buttons
    record_button_->setEnabled(false);
    stop_button_->setEnabled(true);
    muscle_imaging_check_box_->setEnabled(false);

    // Initialize save directory
    save_directory_->initialize();

    // Save the recording metadata into the freshly created save directory.
    // These read the recording parameters directly off the widgets.
    write_experiment_parameters(
        muscle_nominal_exposure_us, muscle_buffer_time_us);
    write_recorder_config();
    write_behavior_calibration_parameters();

    // Send triggering parameters and start recording. The controller reverts to
    // the streaming (revert-to) params when the recording ends.
    TriggerParams rec_params = build_recording_params();
    TriggerParams revert_to_params = build_streaming_params();
    arduino_communication_->start_recording(
        rec_params, revert_to_params, op_sequence);

    // The controller waits camFlushTimeUs after START_RECORDING before it
    // starts triggering, so that frames acquired with the previous (streaming)
    // params drain out of the camera buffers. Ignore frames for a fraction of
    // that window on this side too, so the recording does not begin with stale
    // frames (see camFlushTimeUs in comm_protocol/protocol.h).
    std::this_thread::sleep_for(
        std::chrono::microseconds(cam_flush_time_us * 8 / 10));
    program_state_->is_recording.store(true);

    spdlog::info(
        "Recording STARTED: {} recording, muscle imaging {}. Saving to '{}'",
        current_recording_is_scheduled_ ? "scheduled" : "open",
        muscle_imaging_check_box_->isChecked() ? "enabled" : "disabled",
        save_directory_->get_directory().string());
}

bool MainGUIWindow::confirm_or_resolve_save_directory() {
    std::filesystem::path save_dir = save_directory_->get_directory();
    while (std::filesystem::is_directory(save_dir) &&
           !std::filesystem::is_empty(save_dir)) {
        QMessageBox msg_box(this);
        msg_box.setWindowTitle("Directory not empty");
        msg_box.setText(
            QString(
                "The save directory already exists and is non-empty:\n%1\n\n"
                "Overwrite its contents?")
                .arg(QString::fromStdString(save_dir.string())));
        msg_box.setIcon(QMessageBox::Warning);
        QPushButton *auto_increment_button =
            msg_box.addButton("Auto increment", QMessageBox::ActionRole);
        QPushButton *overwrite_button =
            msg_box.addButton("Overwrite", QMessageBox::ActionRole);
        QPushButton *cancel_button =
            msg_box.addButton("Cancel", QMessageBox::ActionRole);
        msg_box.setEscapeButton(cancel_button);
        msg_box.exec();
        if (msg_box.clickedButton() == overwrite_button) {
            std::filesystem::remove_all(save_dir);
            break;
        } else if (msg_box.clickedButton() == auto_increment_button) {
            // Keep incrementing until we land on a free (non-existent or
            // empty) directory, in case the immediately-next number is also
            // already taken.
            save_dir = increment_directory_name(save_dir.string());
            while (std::filesystem::is_directory(save_dir) &&
                   !std::filesystem::is_empty(save_dir)) {
                save_dir = increment_directory_name(save_dir.string());
            }
            directory_line_edit_->setText(
                QString::fromStdString(save_dir.string()));
            save_dir = save_directory_->get_directory();
        } else {
            return false;
        }
    }
    return true;
}

void MainGUIWindow::write_experiment_parameters(
    int muscle_nominal_exposure_us, int muscle_buffer_time_us) {
    std::filesystem::path output_path = save_directory_->get_directory() /
                                        "metadata/experiment_parameters.yaml";
    bool muscle_imaging_enabled = muscle_imaging_check_box_->isChecked();

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "behavior_fps" << YAML::Value
        << behavior_fps_spin_box_->value();
    out << YAML::Key << "muscle_imaging_enabled" << YAML::Value
        << muscle_imaging_enabled;
    out << YAML::Key << "muscle_sync_ratio" << YAML::Value
        << sync_ratio_spin_box_->value();
    // Convert ms to us.
    out << YAML::Key << "behavior_exposure_time_us" << YAML::Value
        << static_cast<int>(behavior_exposure_time_spin_box_->value() * 1000);
    out << YAML::Key << "muscle_light_on_time_us" << YAML::Value
        << static_cast<int>(muscle_light_on_time_spin_box_->value() * 1000);
    // The derived continuous-mode (auto-sequence) timing -- the nominal
    // per-line exposure programmed into the camera and the slack in the
    // common-time window beyond the light-on time -- is only meaningful when
    // imaging muscle.
    if (muscle_imaging_enabled) {
        out << YAML::Key << "muscle_nominal_exposure_us" << YAML::Value
            << muscle_nominal_exposure_us;
        out << YAML::Key << "muscle_buffer_time_us" << YAML::Value
            << muscle_buffer_time_us;
    }
    out << YAML::Key << "experiment_protocol" << YAML::Value
        << experiment_protocol_->toPlainText().toStdString();
    out << YAML::EndMap;

    std::ofstream fout(output_path);
    if (!fout.is_open()) {
        std::string error_message =
            "Failed to open file for writing experiment parameters: " +
            output_path.string();
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    fout << out.c_str();
    fout.close();

    spdlog::info("Saved experiment parameters to '{}'", output_path.string());
}

void MainGUIWindow::write_recorder_config() {
    std::filesystem::path output_path =
        save_directory_->get_directory() / "metadata/recorder_config.yaml";
    recorder_config_.save_to_file(output_path);
    spdlog::info("Saved recorder config to '{}'", output_path.string());
}

void MainGUIWindow::write_behavior_calibration_parameters() {
    std::filesystem::path output_path =
        save_directory_->get_directory() /
        "metadata/calibration_parameters_behavior.yaml";
    behavior_cam_calibration_params_.save_to_file(output_path);
    spdlog::info(
        "Saved behavior calibration parameters to '{}'", output_path.string());
}

void MainGUIWindow::stop_recording() {
    // Slot for the Stop button: a user-initiated stop.
    end_recording(/*reachedProgrammedEnd=*/false);
}

void MainGUIWindow::end_recording(bool reached_programmed_end) {
    record_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    muscle_imaging_check_box_->setEnabled(true);

    if (reached_programmed_end) {
        // A scheduled recording reached its end: the controller already
        // reverted to the streaming params via its opSequence STOP step, so
        // there is nothing to send (a STOP_RECORDING here would fault the
        // controller).
    } else if (current_recording_is_scheduled_) {
        // Manual early abort of a scheduled recording. STOP_RECORDING is only
        // valid for an open recording, so revert by re-streaming instead.
        arduino_communication_->stream(build_streaming_params());
    } else {
        // Open recording: STOP_RECORDING reverts the controller to streaming
        // (using the revert-to params sent with START_RECORDING).
        arduino_communication_->stop_recording();
    }

    // Revert the free-running muscle camera to the streaming muscle frame rate,
    // matching the streaming params the controller was just reverted to.
    push_muscle_camera_exposure(streaming_behavior_fps_, streaming_sync_ratio_);

    // Stop queuing frames. The acquirer threads flush any partial behavior
    // group and discard subsequent frames (see behaviorImageAcquirer).
    program_state_->is_recording.store(false);

    spdlog::info(
        "Recording STOPPED ({}): {} recording, muscle imaging {}",
        reached_programmed_end ? "reached scheduled end"
                               : "user-initiated stop",
        current_recording_is_scheduled_ ? "scheduled" : "open",
        muscle_imaging_check_box_->isChecked() ? "enabled" : "disabled");

    current_recording_is_scheduled_ = false;
}

void MainGUIWindow::push_muscle_camera_exposure(
    int beh_frame_rate, int sync_ratio) {
    // In continuous (auto-sequence) mode the camera free-runs at
    // 1/(nominalExposure + readout), so the nominal per-line exposure sets the
    // frame rate. Pick it so the camera produces muscle frames at
    // behFrameRate / syncRatio. See docs/data_acquisition.md and
    // MuscleTriggerTiming.
    unsigned int muscle_interval_us =
        static_cast<unsigned int>(1000000.0 * sync_ratio / beh_frame_rate);
    int exposure_us = static_cast<int>(muscle_interval_us) -
                      static_cast<int>(pco_cam_readout_time_us_);
    if (exposure_us <= 0) {
        spdlog::error(
            "Cannot set muscle camera exposure: muscle interval ({} us) is not "
            "longer than the sensor readout time ({} us).",
            muscle_interval_us,
            pco_cam_readout_time_us_);
        return;
    }
    muscle_recording_state_->muscle_camera->set_nominal_exposure_us(
        static_cast<unsigned int>(exposure_us));
}

TriggerParams MainGUIWindow::build_streaming_params() const {
    TriggerParams params;
    // During streaming the controller always runs DEFAULT parameters. The
    // recording spin boxes (behavior FPS, sync ratio, behavior exposure, muscle
    // light-on) are buffered in the GUI and take effect only when a recording
    // starts (see buildRecordingParams). This keeps the controller's status
    // display showing the actual live behavior, never the not-yet-executed
    // recording config.
    //
    // The one live streaming control is the muscle-imaging checkbox: it toggles
    // whether the behavior camera is locked to the muscle camera and the blue
    // excitation LED is pulsed (enableMuscle), for muscle preview. When false,
    // the muscle-only fields are still sent but ignored by the controller.
    params.enable_muscle = muscle_imaging_enabled_;
    params.beh_frame_rate = streaming_behavior_fps_;
    params.beh_musc_sync_ratio = streaming_sync_ratio_;
    params.beh_exp_time = static_cast<unsigned int>(default_beh_exp_time_us_);
    params.musc_eff_exp_time =
        static_cast<unsigned int>(default_musc_light_on_time_us_);
    params.pco_cam_rolling_time = pco_cam_rolling_time_us_;
    params.pco_cam_readout_time = pco_cam_readout_time_us_;
    return params;
}

TriggerParams MainGUIWindow::build_recording_params() const {
    TriggerParams params;
    // The muscle camera is recorded (and the blue excitation light pulsed) only
    // when muscle imaging is enabled; otherwise the controller free-runs the
    // behavior camera (enableMuscle = false).
    params.enable_muscle = muscle_imaging_enabled_;
    params.beh_frame_rate = behavior_fps_spin_box_->value();
    params.beh_musc_sync_ratio = sync_ratio_spin_box_->value();
    params.beh_exp_time = static_cast<unsigned int>(
        behavior_exposure_time_spin_box_->value() * 1000);
    params.musc_eff_exp_time = static_cast<unsigned int>(
        muscle_light_on_time_spin_box_->value() * 1000);
    params.pco_cam_rolling_time = pco_cam_rolling_time_us_;
    params.pco_cam_readout_time = pco_cam_readout_time_us_;
    return params;
}

void MainGUIWindow::closeEvent(QCloseEvent *event) {
    spdlog::info("User is closing GUI window. Quitting gracefully.");
    if (!quit_program()) {
        event->ignore();
        return;
    }
    event->accept();
}

void MainGUIWindow::browse_directory() {
    std::string current_directory = save_directory_->get_directory();
    QString dir = QFileDialog::getExistingDirectory(
        this,
        "Open Directory",
        QString::fromStdString(current_directory),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) {
        directory_line_edit_->setText(dir);

        save_directory_->set_directory(dir.toStdString());
        spdlog::info("Directory changed to '{}'", dir.toStdString());
    } else {
        spdlog::error("Directory is an empty string; failed to open.");
    }
}

void MainGUIWindow::increment_directory() {
    // Increment at least once, then keep incrementing until we land on a free
    // (non-existent or empty) directory, matching the auto-increment behavior
    // used when starting a recording (see confirmOrResolveSaveDirectory).
    std::filesystem::path incremented =
        increment_directory_name(save_directory_->get_directory().string());
    while (std::filesystem::is_directory(incremented) &&
           !std::filesystem::is_empty(incremented)) {
        incremented = increment_directory_name(incremented.string());
    }
    directory_line_edit_->setText(QString::fromStdString(incremented.string()));
}

cv::Mat add_corner_marker(
    const cv::Mat &image,
    double arena_size_x_mm,
    double arena_size_y_mm,
    MotionStagePosition stage_position,
    const CalibrationParams &behavior_cam_calibration_params) {
    cv::Mat image_for_display = image.clone();
    assert(image_for_display.size() == image.size());

    std::vector<std::tuple<double, double>> corner_positions = {
        {0.0, 0.0},
        {arena_size_x_mm, 0.0},
        {arena_size_x_mm, arena_size_y_mm},
        {0.0, arena_size_y_mm}};

    std::vector<cv::Point> pixel_points;
    for (auto [x, y] : corner_positions) {
        int pixel_row, pixel_col;
        std::tie(pixel_row, pixel_col) =
            behavior_cam_calibration_params
                .stage_pos_and_physical_pos_to_pixel_pos(
                    stage_position.x_pos_mm, stage_position.y_pos_mm, x, y);
        pixel_points.emplace_back(pixel_col, pixel_row);
        cv::circle(
            image_for_display,
            cv::Point(pixel_col, pixel_row),
            5,
            cv::Scalar(255, 255, 255),
            -1);
    }
    for (size_t i = 0; i < pixel_points.size(); ++i) {
        cv::line(
            image_for_display,
            pixel_points[i],
            pixel_points[(i + 1) % pixel_points.size()],
            cv::Scalar(255, 0, 0),
            2);
    }

    return image_for_display;
}

void MainGUIWindow::update_behavior_image_display() {
    cv::Mat latest_frame =
        behavior_recording_state_->latest_frame_holder->get_latest_frame_data()
            .image;
    if (latest_frame.empty()) {
        return;
    }
    cv::Mat corrected_frame;
    reorient_behavior_image(latest_frame, corrected_frame);

    MotionStagePosition my_stage_position;
    {
        std::lock_guard<std::mutex> lock(
            tracking_control_state_->latest_motion_stage_position_mutex);
        my_stage_position =
            tracking_control_state_->latest_motion_stage_position;
    }

    // Warp the active-area mask into camera-image space and convert the
    // grayscale frame to BGR and tint out-of-arena pixels red at 50% opacity
    // for visualization.
    cv::Mat active_mask_curr_view = active_area_mask_.warp_to_current_view(
        corrected_frame, my_stage_position);
    cv::Mat bgr_image;
    cv::cvtColor(corrected_frame, bgr_image, cv::COLOR_GRAY2BGR);
    cv::Mat outside_arena;
    cv::threshold(
        active_mask_curr_view, outside_arena, 0, 255, cv::THRESH_BINARY_INV);
    std::vector<cv::Mat> channels(3);
    cv::split(bgr_image, channels);
    // Red tint at 50% opacity: new_red = curr + (255 - curr) / 2
    cv::Mat inv, half_inv, tinted_red;
    cv::subtract(cv::Scalar(255), channels[2], inv);
    cv::divide(inv, 2, half_inv);
    cv::add(channels[2], half_inv, tinted_red);
    tinted_red.copyTo(channels[2], outside_arena); // red channel (BGR)
    cv::merge(channels, bgr_image);

    cv::Mat image_for_display = add_corner_marker(
        bgr_image,
        active_area_mask_.arena_width_mm,
        active_area_mask_.arena_height_mm,
        my_stage_position,
        behavior_cam_calibration_params_);

    QImage q_image = cv_mat_to_q_image(image_for_display);
    QPixmap pixmap = QPixmap::fromImage(q_image).scaled(
        behavior_image_display_label_->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation);
    behavior_image_display_label_->setPixmap(pixmap);
}

void MainGUIWindow::update_muscle_image_display() {
    if (!muscle_imaging_enabled_) {
        muscle_image_display_label_->clear();
        return;
    }

    cv::Mat latest_frame =
        muscle_recording_state_->latest_frame_holder->get_latest_frame_data()
            .image;
    if (latest_frame.empty()) {
        return;
    }
    // Update the live histogram from the raw 16-bit frame, then normalize the
    // preview using the [vmin, vmax] window selected on the slider.
    muscle_histogram_widget_->set_image(latest_frame);
    cv::Mat processed_frame;
    convert16_bit_to8_bit(
        latest_frame,
        processed_frame,
        muscle_histogram_widget_->vmin(),
        muscle_histogram_widget_->vmax());
    reorient_muscle_image(processed_frame, processed_frame);
    QImage q_image = cv_mat_to_q_image(processed_frame);
    QPixmap pixmap = QPixmap::fromImage(q_image).scaled(
        muscle_image_display_label_->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation);
    muscle_image_display_label_->setPixmap(pixmap);
}

int parse_protocol_string(
    const std::string &protocol_text_field_string,
    std::deque<OperationStep> &op_sequence)
/**
 * Parse the experiment-protocol text field into an opSequence. The text is a
 * ";"-separated list of steps, each "frameIdx/channel/op":
 *   - "<n>/ch2/on", "<n>/ch3/off": toggle an optogenetics channel
 *   - "<n>/x/stop": end the recording and revert to streaming
 * An empty string (or a single ";") denotes an open recording with no
 * programmed steps. If any steps are given, the protocol must contain exactly
 * one "<n>/x/stop" step and it must be the very last step. Returns the number
 * of steps, or -1 on a malformed string.
 */
{
    op_sequence.clear();

    auto report_error = []() {
        spdlog::error("Invalid experiment protocol string");
        QMessageBox::critical(
            nullptr,
            "Invalid experiment protocol",
            "The experiment protocol string is invalid.\n\n"
            "It must be a ';'-separated list of steps, each of the form\n"
            "    frameIdx/channel/op\n"
            "where:\n"
            "  - frameIdx is a non-negative integer: the behavior-frame index "
            "after which the step is applied;\n"
            "  - to switch an optogenetics channel, channel is 'ch2' or 'ch3' "
            "(channel 1 is reserved for the IR LED) and op is 'on' or 'off';\n"
            "  - to end the recording, channel is 'x' and op is 'stop'.\n\n"
            "If any steps are given, the protocol must contain exactly one "
            "'x/stop' step, and it must be the very last step.\n\n"
            "Do not add a trailing ';' at the end.\n\n"
            "Leave empty for open recording (no programmed stop).\n\n"
            "Example:\n"
            "    300/ch2/on;600/ch2/off;900/x/stop\n"
            "turns channel 2 on after frame 300, off after frame 600, and "
            "stops the recording after frame 900.");
        return -1;
    };

    if (protocol_text_field_string == ";") {
        return 0;
    }

    std::istringstream stream(protocol_text_field_string);
    std::string token;
    int num_steps_parsed = 0;
    while (std::getline(stream, token, ';')) {
        if (token.empty()) {
            return report_error();
        }

        std::istringstream token_stream(token);
        std::string frame_str, channel_str, op_str;
        if (!std::getline(token_stream, frame_str, '/') ||
            !std::getline(token_stream, channel_str, '/') ||
            !std::getline(token_stream, op_str, '/')) {
            return report_error();
        }

        unsigned long frame_idx = 0;
        OptoChannel channel = OptoChannel::all;
        OpType op = OpType::stop;
        try {
            frame_idx = std::stoul(frame_str);
            if (channel_str == "x" && op_str == "stop") {
                channel = OptoChannel::all;
                op = OpType::stop;
            } else if (channel_str.rfind("ch", 0) == 0) {
                channel =
                    static_cast<OptoChannel>(std::stoi(channel_str.substr(2)));
                if (op_str == "on") {
                    op = OpType::on;
                } else if (op_str == "off") {
                    op = OpType::off;
                } else {
                    return report_error();
                }
            } else {
                return report_error();
            }
        } catch (const std::exception &) {
            return report_error();
        }

        OperationStep step(frame_idx, channel, op);
        if (!step.is_valid) {
            return report_error();
        }
        op_sequence.push_back(step);
        ++num_steps_parsed;
    }

    // If any steps are given, require exactly one stop step, at the very end.
    if (num_steps_parsed > 0) {
        int num_stops = 0;
        for (const OperationStep &step : op_sequence) {
            if (step.op == OpType::stop) {
                ++num_stops;
            }
        }
        if (num_stops != 1 || op_sequence.back().op != OpType::stop) {
            return report_error();
        }
    }

    return num_steps_parsed;
}
