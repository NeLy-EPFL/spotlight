#include "recorder/apps/gui.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "recorder/common/stage_timing.h"

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

MotionControlWidget::MotionControlWidget(
    const RecorderConfig &recorder_config,
    std::shared_ptr<TrackingControlState> tracking_control_state,
    double min_x_absolute_mm,
    double max_x_absolute_mm,
    double min_y_absolute_mm,
    double max_y_absolute_mm,
    QWidget *parent)
    : QWidget(parent),
      tracking_control_state_(std::move(tracking_control_state)) {
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

MainGUIWindow::MainGUIWindow(Recorder &recorder, QWidget *parent)
    : QWidget(parent), recorder_(recorder),
      recorder_config_(recorder.recorder_config()) {

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

    // The Recorder starts out streaming (live preview, not saving) with muscle
    // imaging off.
    update_controls(/*recording=*/false);
}

QLayout *MainGUIWindow::create_behavior_fps_row() {
    behavior_fps_spin_box_ = new QSpinBox(this);
    behavior_fps_spin_box_->setRange(1, 1000);
    behavior_fps_spin_box_->setValue(
        recorder_.get_default_recording_params().behavior_fps);
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
    sync_ratio_spin_box_->setValue(
        recorder_.get_default_recording_params().sync_ratio);
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
    behavior_exposure_time_spin_box_->setValue(
        recorder_.get_default_recording_params().behavior_exposure_time_us /
        1000.0);
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
    muscle_light_on_time_spin_box_->setValue(
        recorder_.get_default_recording_params().muscle_light_on_time_us /
        1000.0);
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
    directory_line_edit_->setText(
        recorder_.save_directory()->get_directory().c_str());
    connect(
        directory_line_edit_,
        &QLineEdit::textChanged,
        this,
        [this](const QString &text) {
            spdlog::debug("save_directory changed to {}", text.toStdString());
            recorder_.save_directory()->set_directory(text.toStdString());
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
    image_display_timer_->start(1000 / recorder_.streaming_behavior_fps());
    return behavior_column_layout;
}

QLayout *MainGUIWindow::create_muscle_preview_pane(
    int preview_width, int preview_height) {
    muscle_image_display_label_ = new QLabel(this);
    muscle_image_display_label_->setFixedSize(preview_width, preview_height);

    // Histograms + normalization-range sliders of the calcium (green) and
    // fiducial (red) channels, shown only while muscle imaging is enabled
    // (toggled by the Enable checkbox below). Each camera has its own pixel
    // value range in the recorder config.
    auto make_histogram_widget = [this](
                                     const std::string &role_name,
                                     const QColor &bin_color) {
        auto get_parameter = [this, &role_name](const std::string &key) {
            return recorder_config_.get_parameter<int>(
                "muscle_camera", key + "_" + role_name);
        };
        return new MuscleHistogramWidget(
            get_parameter("histogram_display_min"),
            get_parameter("histogram_display_max"),
            get_parameter("default_display_vmin"),
            get_parameter("default_display_vmax"),
            bin_color,
            this);
    };
    calcium_histogram_widget_ =
        make_histogram_widget("calcium", QColor(80, 200, 80));
    fiducial_histogram_widget_ =
        make_histogram_widget("fiducial", QColor(220, 70, 70));

    // Channel on/off checkboxes. Display only, so they stay usable during
    // recordings.
    calcium_channel_check_box_ = new QCheckBox("Calcium", this);
    calcium_channel_check_box_->setChecked(true);
    fiducial_channel_check_box_ = new QCheckBox("Fiducials", this);
    fiducial_channel_check_box_->setChecked(true);
    behavior_channel_check_box_ = new QCheckBox("Behavior", this);
    behavior_channel_check_box_->setChecked(false);
    // The muscle-imaging widgets, shown only while muscle imaging is enabled
    std::vector<QWidget *> muscle_imaging_widgets = {
        calcium_histogram_widget_,
        fiducial_histogram_widget_,
        calcium_channel_check_box_,
        fiducial_channel_check_box_,
        behavior_channel_check_box_};
    for (QWidget *widget : muscle_imaging_widgets) {
        widget->setVisible(false);
    }
    calcium_histogram_widget_->setFixedWidth(preview_width);
    fiducial_histogram_widget_->setFixedWidth(preview_width);
    calcium_channel_check_box_->setStyleSheet("color: rgb(0, 140, 0)");
    fiducial_channel_check_box_->setStyleSheet("color: rgb(200, 0, 0)");
    behavior_channel_check_box_->setStyleSheet("color: rgb(0, 0, 220)");

    // Muscle-imaging on/off checkbox, right-aligned in the column title so it
    // sits at the right edge of the muscle preview.
    muscle_imaging_check_box_ = new QCheckBox("Enable", this);
    muscle_imaging_check_box_->setChecked(false);
    if (!recorder_.muscle_camera()) {
        muscle_imaging_check_box_->setToolTip(
            "run-spotlight was started without the muscle cameras "
            "(--no-muscle)");
    }
    connect(
        muscle_imaging_check_box_,
        &QCheckBox::checkStateChanged,
        this,
        [this, muscle_imaging_widgets](int state) {
            // The checkbox is disabled during recordings, when muscle imaging
            // must not be toggled (see Recorder::set_muscle_imaging_enabled).
            bool enabled = (state == Qt::Checked);
            recorder_.set_muscle_imaging_enabled(enabled);
            // The muscle-only parameters are editable only when imaging muscle.
            sync_ratio_spin_box_->setEnabled(enabled);
            muscle_light_on_time_spin_box_->setEnabled(enabled);
            // The histograms and channel checkboxes are only meaningful with
            // a live muscle preview.
            for (QWidget *widget : muscle_imaging_widgets) {
                widget->setVisible(enabled);
            }
        });

    // Stack the muscle preview directly on top of its histograms (no gap
    // between them), then place that stack under the column title (title text
    // on the left; the channel checkboxes and, separated from them, the Enable
    // checkbox right-aligned to the preview's right edge).
    QVBoxLayout *muscle_preview_stack = new QVBoxLayout();
    muscle_preview_stack->setSpacing(0);
    muscle_preview_stack->setContentsMargins(0, 0, 0, 0);
    muscle_preview_stack->addWidget(muscle_image_display_label_);
    muscle_preview_stack->addWidget(calcium_histogram_widget_);
    muscle_preview_stack->addWidget(fiducial_histogram_widget_);
    QHBoxLayout *muscle_title_layout = new QHBoxLayout();
    muscle_title_layout->setContentsMargins(0, 0, 0, 0);
    muscle_title_layout->addWidget(new QLabel("Muscle preview", this));
    muscle_title_layout->addStretch();
    muscle_title_layout->addWidget(calcium_channel_check_box_);
    muscle_title_layout->addWidget(fiducial_channel_check_box_);
    muscle_title_layout->addWidget(behavior_channel_check_box_);
    muscle_title_layout->addSpacing(20);
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
        static_cast<float>(recorder_.streaming_behavior_fps()) /
        recorder_.streaming_sync_ratio();
    spdlog::info("Muscle streaming FPS: {}", muscle_streaming_fps);
    muscle_image_display_timer->start(1000 / muscle_streaming_fps);
    return muscle_column_layout;
}

QLayout *MainGUIWindow::create_stage_preview_pane() {
    motion_control_widget_ = new MotionControlWidget(
        recorder_config_,
        recorder_.tracking_control_state(),
        recorder_.stage_range().min_x_mm,
        recorder_.stage_range().max_x_mm,
        recorder_.stage_range().min_y_mm,
        recorder_.stage_range().max_y_mm,
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
                recorder_.tracking_control_state()->tracking_on.store(true);
            } else {
                recorder_.tracking_control_state()->tracking_on.store(false);
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
    // The muscle column is the muscle preview stacked on top of its two
    // histograms / sliders; size the behavior preview to that combined height
    // so the two columns line up.
    int muscle_column_height =
        muscle_camera_preview_height + 2 * histogram_widget_height;

    // Derive the behavior preview width from the displayed behavior frame's
    // aspect ratio, so the image fills the label exactly with no left/right
    // padding (which would otherwise widen the gap to the muscle preview beyond
    // the muscle-to-stage gap). The behavior frame is rotated 90 degrees for
    // display (see reorient_behavior_image), so its displayed width:height
    // ratio is the ROI height:width.
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
    // by themselves; the Recorder then reverts the cameras to streaming, and
    // this reverts the UI.
    QTimer *programmed_stop_check_timer = new QTimer(this);
    connect(programmed_stop_check_timer, &QTimer::timeout, this, [this]() {
        if (recorder_.finalize_programmed_stop_if_reached()) {
            update_controls(/*recording=*/false);
            QMessageBox::information(
                this,
                "Recording stopped",
                "End of protocol reached. Recording stopped.");
        }
    });
    programmed_stop_check_timer->start(500); // Check every 0.5 second
}

RecordingParams MainGUIWindow::get_recording_params() const {
    RecordingParams params;
    params.behavior_fps = behavior_fps_spin_box_->value();
    params.sync_ratio = sync_ratio_spin_box_->value();
    // Convert ms to us.
    params.behavior_exposure_time_us =
        static_cast<int>(behavior_exposure_time_spin_box_->value() * 1000);
    params.muscle_light_on_time_us =
        static_cast<int>(muscle_light_on_time_spin_box_->value() * 1000);
    params.experiment_protocol =
        experiment_protocol_->toPlainText().toStdString();
    return params;
}

void MainGUIWindow::start_recording() {
    // Checked before resolving the save directory, which may delete files
    if (!recorder_.is_behavior_camera_ready()) {
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

    if (std::optional<std::string> error =
            recorder_.start_recording(get_recording_params())) {
        QMessageBox::critical(this, "Error", QString::fromStdString(*error));
        return;
    }

    update_controls(/*recording=*/true);
}

void MainGUIWindow::profiling_step(int step) {
    spdlog::info("Profiling step {}", step);
    // Save what the previews show, for a visual check
    behavior_image_display_label_->pixmap().save(
        QString("preview_behavior_step%1.png").arg(step));
    muscle_image_display_label_->pixmap().save(
        QString("preview_muscle_step%1.png").arg(step));
    if (step == 1) {
        muscle_imaging_check_box_->setChecked(true);
    } else if (step == 2) {
        start_recording();
    } else if (step == 3) {
        stop_recording();
    }
}

void MainGUIWindow::update_controls(bool recording) {
    record_button_->setEnabled(!recording);
    stop_button_->setEnabled(recording);
    muscle_imaging_check_box_->setEnabled(
        !recording && recorder_.muscle_camera() != nullptr);
}

bool MainGUIWindow::confirm_or_resolve_save_directory() {
    std::filesystem::path save_dir = recorder_.save_directory()->get_directory();
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
            save_dir = recorder_.save_directory()->get_directory();
        } else {
            return false;
        }
    }
    return true;
}

void MainGUIWindow::stop_recording() {
    // Slot for the Stop button: a user-initiated stop.
    recorder_.end_recording(/*reached_programmed_end=*/false);
    update_controls(/*recording=*/false);
}

void MainGUIWindow::closeEvent(QCloseEvent * /*event*/) {
    spdlog::info("User is closing GUI window. Quitting gracefully.");
    recorder_.quit();
}

void MainGUIWindow::browse_directory() {
    std::string current_directory = recorder_.save_directory()->get_directory();
    QString dir = QFileDialog::getExistingDirectory(
        this,
        "Open Directory",
        QString::fromStdString(current_directory),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) {
        directory_line_edit_->setText(dir);

        recorder_.save_directory()->set_directory(dir.toStdString());
        spdlog::info("Directory changed to '{}'", dir.toStdString());
    } else {
        spdlog::error("Directory is an empty string; failed to open.");
    }
}

void MainGUIWindow::increment_directory() {
    // Increment at least once, then keep incrementing until we land on a free
    // (non-existent or empty) directory, matching the auto-increment behavior
    // used when starting a recording (see confirm_or_resolve_save_directory).
    std::filesystem::path incremented =
        increment_directory_name(
            recorder_.save_directory()->get_directory().string());
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
    const CalibrationParams &behavior_cam_calibration_params,
    double view_scale) {
    cv::Mat image_for_display = image.clone();
    // Marker sizes at full resolution, scaled with the image (at least 1 px)
    const int dot_radius = std::max(1, static_cast<int>(std::lround(5 * view_scale)));
    const int line_thickness =
        std::max(1, static_cast<int>(std::lround(2 * view_scale)));
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
        cv::Point point(
            static_cast<int>(std::lround(pixel_col * view_scale)),
            static_cast<int>(std::lround(pixel_row * view_scale)));
        pixel_points.push_back(point);
        cv::circle(
            image_for_display, point, dot_radius, cv::Scalar(255, 255, 255), -1);
    }
    for (size_t i = 0; i < pixel_points.size(); ++i) {
        cv::line(
            image_for_display,
            pixel_points[i],
            pixel_points[(i + 1) % pixel_points.size()],
            cv::Scalar(255, 0, 0),
            line_thickness);
    }

    return image_for_display;
}

void MainGUIWindow::update_behavior_image_display() {
    // Only profile ticks during a recording
    auto record = [this](const char *stage, int64_t us) {
        if (recorder_.is_recording()) {
            stage_timing_record(stage, us);
        }
    };
    uint64_t t0 = get_current_time_microseconds();
    FrameData latest_frame_data =
        recorder_.behavior_recording_state()->latest_frame_holder->get_latest_frame_data();
    cv::Mat latest_frame = latest_frame_data.image;
    if (latest_frame.empty()) {
        return;
    }
    // Downscale to the preview size first so that everything below works on a
    // small image (several times cheaper than processing the full frame and
    // scaling at the end). The frame is rotated by 90 degrees for display, so
    // its displayed width is its row count.
    QSize label_size = behavior_image_display_label_->size();
    double view_scale = std::min(
        static_cast<double>(label_size.width()) / latest_frame.rows,
        static_cast<double>(label_size.height()) / latest_frame.cols);
    cv::Size preview_size(
        cvRound(latest_frame.cols * view_scale),
        cvRound(latest_frame.rows * view_scale));
    // Halving first lets INTER_AREA use its fast integer-factor path (~3x
    // faster overall)
    cv::Mat small_frame = latest_frame;
    if (view_scale < 0.5) {
        cv::resize(
            latest_frame, small_frame,
            cv::Size(latest_frame.cols / 2, latest_frame.rows / 2), 0, 0,
            cv::INTER_AREA);
    }
    cv::resize(small_frame, small_frame, preview_size, 0, 0, cv::INTER_AREA);
    uint64_t t_resize = get_current_time_microseconds();
    cv::Mat corrected_frame;
    reorient_behavior_image(small_frame, corrected_frame);

    MotionStagePosition my_stage_position;
    {
        std::lock_guard<std::mutex> lock(
            recorder_.tracking_control_state()->latest_motion_stage_position_mutex);
        my_stage_position =
            recorder_.tracking_control_state()->latest_motion_stage_position;
    }

    // Convert the grayscale frame to BGR. While tracking, warp the
    // active-area mask into camera-image space and tint out-of-arena pixels
    // red at 50% opacity for visualization.
    cv::Mat bgr_image;
    cv::cvtColor(corrected_frame, bgr_image, cv::COLOR_GRAY2BGR);
    if (tracking_enabled_check_box_->isChecked()) {
        cv::Mat active_mask_curr_view =
            recorder_.active_area_mask().warp_to_current_view(
                corrected_frame, my_stage_position, view_scale);
        cv::Mat outside_arena;
        cv::threshold(
            active_mask_curr_view,
            outside_arena,
            0,
            255,
            cv::THRESH_BINARY_INV);
        std::vector<cv::Mat> channels(3);
        cv::split(bgr_image, channels);
        // Red tint at 50% opacity: new_red = curr + (255 - curr) / 2
        cv::Mat inv, half_inv, tinted_red;
        cv::subtract(cv::Scalar(255), channels[2], inv);
        cv::divide(inv, 2, half_inv);
        cv::add(channels[2], half_inv, tinted_red);
        tinted_red.copyTo(channels[2], outside_arena); // red channel (BGR)
        cv::merge(channels, bgr_image);
    }

    cv::Mat image_for_display = add_corner_marker(
        bgr_image,
        recorder_.active_area_mask().arena_width_mm,
        recorder_.active_area_mask().arena_height_mm,
        my_stage_position,
        recorder_.behavior_cam_calibration_params(),
        view_scale);

    uint64_t t1 = get_current_time_microseconds();
    QImage q_image = cv_mat_to_q_image(image_for_display);
    uint64_t t2 = get_current_time_microseconds();
    QPixmap pixmap = QPixmap::fromImage(q_image);
    uint64_t t3 = get_current_time_microseconds();
    behavior_image_display_label_->setPixmap(pixmap);
    uint64_t t4 = get_current_time_microseconds();
    record("gui beh: downscale to label size (INTER_AREA)", t_resize - t0);
    record("gui beh: OpenCV processing (rotate, color, tint, marker)", t1 - t_resize);
    record("gui beh: cv::Mat -> QImage (BGR->RGB + copy)", t2 - t1);
    record("gui beh: QPixmap::fromImage", t3 - t2);
    record("gui beh: QLabel::setPixmap", t4 - t3);
    record("gui beh: total per tick", t4 - t0);
    record(
        "gui beh: frame age when handed to Qt (host-received -> setPixmap)",
        t4 - latest_frame_data.received_time);
}

void MainGUIWindow::update_muscle_image_display() {
    if (!recorder_.is_muscle_imaging_enabled()) {
        muscle_image_display_label_->clear();
        return;
    }

    auto record = [this](const char *stage, int64_t us) {
        if (recorder_.is_recording()) {
            stage_timing_record(stage, us);
        }
    };
    uint64_t t0 = get_current_time_microseconds();
    FrameData calcium_frame_data =
        recorder_.muscle_recording_state()->latest_calcium_frame_holder->get_latest_frame_data();
    cv::Mat calcium_frame = calcium_frame_data.image;
    cv::Mat fiducial_frame =
        recorder_.muscle_recording_state()->latest_fiducial_frame_holder
            ->get_latest_frame_data()
            .image;
    if (calcium_frame.empty() || fiducial_frame.empty()) {
        return;
    }
    // Update the live histograms from the raw 16-bit frames, then normalize
    // each channel using the [vmin, vmax] window selected on its slider.
    cv::Mat calcium_channel =
        calcium_histogram_widget_->update_and_normalize(calcium_frame);
    cv::Mat fiducial_channel =
        fiducial_histogram_widget_->update_and_normalize(fiducial_frame);
    uint64_t t_hist = get_current_time_microseconds();
    record("gui mus: 2x histogram + 16->8 bit normalize", t_hist - t0);
    reorient_muscle_image(calcium_channel, calcium_channel);
    reorient_muscle_image(fiducial_channel, fiducial_channel);
    // Everything is displayed in the calcium camera's view
    const cv::Size view_size = calcium_channel.size();
    fiducial_channel = warp_to_calcium_view(fiducial_channel, view_size);
    cv::Mat behavior_channel = cv::Mat::zeros(view_size, CV_8UC1);
    if (behavior_channel_check_box_->isChecked()) {
        cv::Mat behavior_frame =
            recorder_.behavior_recording_state()->latest_frame_holder
                ->get_latest_frame_data()
                .image;
        if (!behavior_frame.empty()) {
            cv::Mat reoriented_behavior_frame;
            reorient_behavior_image(behavior_frame, reoriented_behavior_frame);
            behavior_channel =
                warp_to_calcium_view(reoriented_behavior_frame, view_size);
        }
    }
    if (!calcium_channel_check_box_->isChecked()) {
        calcium_channel.setTo(0);
    }
    if (!fiducial_channel_check_box_->isChecked()) {
        fiducial_channel.setTo(0);
    }

    cv::Mat bgr_image;
    cv::merge(
        std::vector<cv::Mat>{behavior_channel, calcium_channel, fiducial_channel},
        bgr_image);
    uint64_t t1 = get_current_time_microseconds();
    QImage q_image = cv_mat_to_q_image(bgr_image);
    QPixmap pixmap = QPixmap::fromImage(q_image).scaled(
        muscle_image_display_label_->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation);
    uint64_t t2 = get_current_time_microseconds();
    muscle_image_display_label_->setPixmap(pixmap);
    uint64_t t3 = get_current_time_microseconds();
    record("gui mus: rotate, warp, merge channels", t1 - t_hist);
    record("gui mus: QImage + QPixmap + scaled", t2 - t1);
    record("gui mus: QLabel::setPixmap", t3 - t2);
    record("gui mus: total per tick", t3 - t0);
    record(
        "gui mus: frame age when handed to Qt (recorder-received -> setPixmap)",
        t3 - calcium_frame_data.received_time);
}

cv::Mat
warp_to_calcium_view(const cv::Mat &image, const cv::Size &calcium_view_size) {
    // Translation that aligns the image centers
    cv::Mat transform =
        (cv::Mat_<double>(2, 3) << 1,
         0,
         (calcium_view_size.width - image.cols) / 2.0,
         0,
         1,
         (calcium_view_size.height - image.rows) / 2.0);
    cv::Mat warped_image;
    cv::warpAffine(image, warped_image, transform, calcium_view_size);
    return warped_image;
}
