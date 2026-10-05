#include "recorder/apps/align_cameras.h"

#define BEHAVIOR_DISPLAY_DOWNSAMPLE_FACTOR 3
#define MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR 4

namespace {
// Interval between display updates
constexpr int display_update_interval_ms = 40;
// Spacing between the camera panels, and between a panel's title and its
// instructions (px)
constexpr int panel_spacing = 20;
constexpr int title_spacing = 2;
// Intensity range of the muscle histograms and their range sliders: the
// cameras' full 16-bit range
constexpr int muscle_histogram_min = 0;
constexpr int muscle_histogram_max = 65535;

using pco_shared_memory::MuscleCameraRole;

// The two muscle cameras, in the order they are shown (left to right)
const std::array<MuscleCameraRole, 2> muscle_camera_roles = {
    MuscleCameraRole::calcium, MuscleCameraRole::fiducial};
// Titles and instructions shown above the images
const QString behavior_camera_title = "Behavior camera";
const QString behavior_camera_instructions =
    "Step 1: Place the bullseye pattern on the arena. Using the knobs on the "
    "Zaber motion controller, move the linear stages until the center of the "
    "bullseye is on the crosshair.";
// Indexed like muscle_camera_roles
const std::array<QString, 2> muscle_camera_titles = {
    "Muscle calcium camera", "Muscle fiducials camera"};
const std::array<QString, 2> muscle_camera_instructions = {
    "Step 2: Click the center of the bullseye. A red dot marks the click, and "
    "a blue dot and box show the nearest valid field of view. If the blue box "
    "is not fully inside the image, physically adjust the camera until it is. "
    "Use the sliders on the histogram below to adjust the pixel value range "
    "for display.",
    "Step 3: Repeat Step 2 for the fiducials camera."};

int full_muscle_image_width;
int full_muscle_image_height;
int muscle_image_roi_width;
int muscle_image_roi_height;
// User-selected ROI center of each muscle camera (indexed like
// muscle_camera_roles), in its image's display coordinates; (-1, -1) until
// the user clicks on that image
std::array<cv::Point, 2> selected_centers_display = {
    cv::Point(-1, -1), cv::Point(-1, -1)};
std::filesystem::path profile_dir;
std::unique_ptr<ArduinoCommunication> arduino_communication = nullptr;

std::filesystem::path get_roi_file_path() {
    return profile_dir / "muscle_camera_roi.yaml";
}

std::tuple<int, int>
display_to_camera_sensor_coords(int display_x, int display_y) {
    // Reverse the 90 degrees counterclockwise rotation and scale back up
    int sensor_x =
        full_muscle_image_width - (display_y * MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR) - 1;
    int sensor_y = display_x * MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR;
    return std::make_tuple(sensor_x, sensor_y);
}

std::tuple<int, int>
camera_sensor_to_display_coords(int sensor_x, int sensor_y) {
    // This is the opposite of display_to_camera_sensor_coords
    int display_x = sensor_y / MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR;
    int display_y =
        (full_muscle_image_width - sensor_x - 1) / MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR;
    return std::make_tuple(display_x, display_y);
}

MuscleCameraROI
get_roi_from_display_center(int x_center_display, int y_center_display) {
    auto [muscle_camera_center_x_sensor, muscle_camera_center_y_sensor] =
        display_to_camera_sensor_coords(x_center_display, y_center_display);
    int x_offset = muscle_camera_center_x_sensor - (muscle_image_roi_width / 2);
    int y_offset =
        muscle_camera_center_y_sensor - (muscle_image_roi_height / 2);
    // PCO ROI quantization: x offsets must be multiples of 32, y offsets
    // multiples of 8.
    int x0 = round_to_nearest_valid_muscle_cam_horizontal(x_offset) + 1;
    int x1 = (x0 - 1) + muscle_image_roi_width;
    int y0 = round_to_nearest_valid_muscle_cam_vertical(y_offset) + 1;
    int y1 = (y0 - 1) + muscle_image_roi_height;

    MuscleCameraROI roi(x0, x1, y0, y1);
    return roi;
}

void draw_muscle_image_roi(cv::Mat &image, const cv::Point &center_display) {
    if (center_display.x == -1) {
        // User didn't select a center point yet
        return;
    }

    // Draw user-selected center point
    cv::Scalar red_color(0, 0, 255);
    cv::circle(image, center_display, 5, red_color, -1);

    // Figure out center point coords on the camera sensor
    MuscleCameraROI roi =
        get_roi_from_display_center(center_display.x, center_display.y);

    // Draw closest feasible center point
    auto [x_center_sensor_actual, y_center_sensor_actual] = roi.get_center_xy();
    auto [x_center_display_actual, y_center_display_actual] =
        camera_sensor_to_display_coords(
            x_center_sensor_actual, y_center_sensor_actual);
    cv::Scalar blue_color(255, 0, 0);
    cv::Point point_actual(x_center_display_actual, y_center_display_actual);
    cv::circle(image, point_actual, 3, blue_color, -1);

    // Draw ROI rectangle
    auto [display_x0, display_y0] =
        camera_sensor_to_display_coords(roi.x0, roi.y0);
    auto [display_x1, display_y1] =
        camera_sensor_to_display_coords(roi.x1, roi.y1);
    cv::rectangle(
        image,
        cv::Point(display_x0, display_y0),
        cv::Point(display_x1, display_y1),
        blue_color,
        /*thickness=*/2);
}

void select_roi_center(size_t camera_idx, int x, int y) {
    selected_centers_display[camera_idx] = cv::Point(x, y);

    // Convert display coordinates to camera sensor coordinates
    auto [sensor_x, sensor_y] = display_to_camera_sensor_coords(x, y);
    spdlog::info(
        "{} camera center point set at (x={}, y={}) on the displayed image, "
        "which translates to (x={}, y={}) on the camera sensor.",
        pco_shared_memory::role_to_string(muscle_camera_roles[camera_idx]),
        x,
        y,
        sensor_x,
        sensor_y);
}

void add_cross_to_behavior_image(cv::Mat &image) {
    // Add a cross to the middle of the behavior image to help with alignment
    cv::line(
        image,
        cv::Point(image.cols / 2, 0),
        cv::Point(image.cols / 2, image.rows),
        cv::Scalar(255, 255, 255),
        1);
    cv::line(
        image,
        cv::Point(0, image.rows / 2),
        cv::Point(image.cols, image.rows / 2),
        cv::Scalar(255, 255, 255),
        1);
}

std::string try_save_selected_rois() {
    // Validate the user-selected ROI centers and, if both are set and in
    // bounds, write both cameras' ROIs to <profile_dir>/muscle_camera_roi.yaml
    // (one section per camera). Returns an empty string when the ROIs were
    // saved, and the reason otherwise.
    std::vector<MuscleCameraROI> rois;
    for (size_t i = 0; i < muscle_camera_roles.size(); ++i) {
        std::string role_name =
            pco_shared_memory::role_to_string(muscle_camera_roles[i]);
        const cv::Point &center = selected_centers_display[i];
        if (center.x == -1) {
            return fmt::format(
                "ROI center of the {} camera not set yet. Please click the "
                "center point on both muscle camera images before saving the "
                "ROIs.",
                role_name);
        }
        MuscleCameraROI roi = get_roi_from_display_center(center.x, center.y);
        if (!roi.is_within_bound(
                full_muscle_image_width, full_muscle_image_height)) {
            return fmt::format(
                "Selected ROI of the {} camera out of bound. Please try again.",
                role_name);
        }
        rois.push_back(roi);
    }

    for (size_t i = 0; i < muscle_camera_roles.size(); ++i) {
        const MuscleCameraROI &roi = rois[i];
        spdlog::info(
            "{} camera ROI: x0={}, x1={}, y0={}, y1={}",
            pco_shared_memory::role_to_string(muscle_camera_roles[i]),
            roi.x0,
            roi.x1,
            roi.y0,
            roi.y1);
    }
    std::filesystem::path roi_file_path = get_roi_file_path();
    if (MuscleCameraROIs{rois[0], rois[1]}.to_file(roi_file_path) != 0) {
        return fmt::format("Failed to open {}", roi_file_path.string());
    }
    spdlog::info("Saved the muscle camera ROIs to {}", roi_file_path.string());
    return "";
}

} // namespace

void ClickableImageLabel::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton && on_click) {
        on_click(
            static_cast<int>(event->position().x()),
            static_cast<int>(event->position().y()));
    }
}

AlignCamerasWindow::AlignCamerasWindow(
    const RecorderConfig &recorder_config,
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
    std::shared_ptr<MuscleRecordingState> muscle_recording_state,
    QWidget *parent)
    : QWidget(parent),
      behavior_recording_state_(std::move(behavior_recording_state)),
      muscle_frame_holders_(
          {muscle_recording_state->latest_calcium_frame_holder,
           muscle_recording_state->latest_fiducial_frame_holder}) {
    setWindowTitle("Align cameras");

    // Initial muscle display range, separate from the main GUI's defaults
    int muscle_display_vmin = recorder_config.get_parameter<int>(
        "muscle_camera", "align_cameras_display_vmin");
    int muscle_display_vmax = recorder_config.get_parameter<int>(
        "muscle_camera", "align_cameras_display_vmax");

    // One column per camera: bold title, instructions, image (and histogram
    // for the muscle cameras). A grid keeps the images aligned when the
    // instructions wrap to different numbers of lines.
    QGridLayout *images_layout = new QGridLayout();
    images_layout->setHorizontalSpacing(panel_spacing);
    auto add_header = [&](int column,
                          const QString &title,
                          const QString &instructions,
                          int width) {
        QLabel *title_label = new QLabel(title, this);
        QFont font = title_label->font();
        // Semi-bold: the default UI font's (Ubuntu) bold is very heavy
        font.setWeight(QFont::DemiBold);
        title_label->setFont(font);
        QLabel *instructions_label = new QLabel(instructions, this);
        instructions_label->setWordWrap(true);
        instructions_label->setFixedWidth(width);
        QVBoxLayout *header_layout = new QVBoxLayout();
        header_layout->setSpacing(title_spacing);
        header_layout->addWidget(title_label);
        header_layout->addWidget(instructions_label);
        header_layout->addStretch();
        images_layout->addLayout(header_layout, 0, column);
    };

    // Images are shown unscaled at the top left of their labels, so that
    // clicks are in display image coordinates. Width and height are swapped
    // because the images are rotated.
    int behavior_image_display_width =
        recorder_config.get_parameter<int>("behavior_camera", "roi_height") /
        BEHAVIOR_DISPLAY_DOWNSAMPLE_FACTOR;
    behavior_label_ = new QLabel(this);
    behavior_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    behavior_label_->setFixedSize(
        behavior_image_display_width,
        recorder_config.get_parameter<int>("behavior_camera", "roi_width") /
            BEHAVIOR_DISPLAY_DOWNSAMPLE_FACTOR);
    add_header(
        0,
        behavior_camera_title,
        behavior_camera_instructions,
        behavior_image_display_width);
    images_layout->addWidget(behavior_label_, 1, 0, 2, 1, Qt::AlignTop);

    int muscle_image_display_width =
        full_muscle_image_height / MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR;
    int muscle_image_display_height =
        full_muscle_image_width / MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR;
    for (size_t i = 0; i < muscle_camera_roles.size(); ++i) {
        muscle_labels_[i] = new ClickableImageLabel();
        muscle_labels_[i]->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        muscle_labels_[i]->setFixedSize(
            muscle_image_display_width, muscle_image_display_height);
        muscle_labels_[i]->on_click = [i](int x, int y) {
            select_roi_center(i, x, y);
        };
        histogram_widgets_[i] = new MuscleHistogramWidget(
            muscle_histogram_min,
            muscle_histogram_max,
            muscle_display_vmin,
            muscle_display_vmax);
        histogram_widgets_[i]->setFixedWidth(muscle_image_display_width);

        int column = static_cast<int>(i) + 1;
        add_header(
            column,
            muscle_camera_titles[i],
            muscle_camera_instructions[i],
            muscle_image_display_width);
        images_layout->addWidget(muscle_labels_[i], 1, column);
        images_layout->addWidget(
            histogram_widgets_[i], 2, column, Qt::AlignTop);
    }

    QPushButton *save_button = new QPushButton("Save ROIs", this);
    QPushButton *quit_button = new QPushButton("Cancel", this);
    connect(save_button, &QPushButton::clicked, this, [this]() {
        std::string error = try_save_selected_rois();
        if (!error.empty()) {
            spdlog::error(error);
            QMessageBox::warning(
                this, "Cannot save ROIs", QString::fromStdString(error));
            return;
        }
        QMessageBox::information(
            this,
            "ROIs saved",
            QString::fromStdString(fmt::format(
                "Saved the muscle camera ROIs to {}",
                get_roi_file_path().string())));
        close();
    });
    connect(quit_button, &QPushButton::clicked, this, &QWidget::close);
    QHBoxLayout *buttons_layout = new QHBoxLayout();
    buttons_layout->addStretch();
    buttons_layout->addWidget(save_button);
    buttons_layout->addWidget(quit_button);

    QVBoxLayout *main_layout = new QVBoxLayout(this);
    main_layout->addLayout(images_layout);
    main_layout->addLayout(buttons_layout);

    connect(&timer_, &QTimer::timeout, this, [this]() { update_images(); });
    timer_.start(display_update_interval_ms);
}

void AlignCamerasWindow::update_images() {
    // Images are empty until the cameras deliver their first frames
    cv::Mat behavior_image =
        behavior_recording_state_->latest_frame_holder->get_latest_frame_data()
            .image;
    if (!behavior_image.empty()) {
        cv::Mat display;
        cv::resize(
            behavior_image,
            display,
            cv::Size(
                behavior_image.cols / BEHAVIOR_DISPLAY_DOWNSAMPLE_FACTOR,
                behavior_image.rows / BEHAVIOR_DISPLAY_DOWNSAMPLE_FACTOR));
        reorient_behavior_image(display, display);
        // Add a cross to the middle of the behavior image to help with
        // alignment
        add_cross_to_behavior_image(display);
        behavior_label_->setPixmap(
            QPixmap::fromImage(cv_mat_to_q_image(display)));
    }

    for (size_t i = 0; i < muscle_camera_roles.size(); ++i) {
        cv::Mat image = muscle_frame_holders_[i]->get_latest_frame_data().image;
        if (image.empty()) {
            continue;
        }
        cv::Mat display = histogram_widgets_[i]->update_and_normalize(image);
        cv::resize(
            display,
            display,
            cv::Size(
                display.cols / MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR,
                display.rows / MUSCLE_DISPLAY_DOWNSAMPLE_FACTOR));
        reorient_muscle_image(display, display);
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);

        // Draw markers on the displayed image to indicate ROI
        draw_muscle_image_roi(display, selected_centers_display[i]);
        muscle_labels_[i]->setPixmap(
            QPixmap::fromImage(cv_mat_to_q_image(display)));
    }
}

void align_camera(const std::filesystem::path &profile_dir) {
    std::filesystem::path config_path = profile_dir / "recorder_config.yaml";
    spdlog::info(
        "align-cameras loading recorder configuration from {}",
        config_path.string());
    RecorderConfig recorder_config(config_path);

    // Load muscle camera full frame size
    full_muscle_image_width =
        recorder_config.get_parameter<int>("muscle_camera", "full_frame_width");
    full_muscle_image_height = recorder_config.get_parameter<int>(
        "muscle_camera", "full_frame_height");
    muscle_image_roi_width =
        recorder_config.get_parameter<int>("muscle_camera", "roi_width");
    muscle_image_roi_height =
        recorder_config.get_parameter<int>("muscle_camera", "roi_height");

    // Set up shared recording states
    std::shared_ptr<ProgramState> program_state =
        std::make_shared<ProgramState>();
    std::shared_ptr<ProgrammedStop> programmed_recording_stop =
        std::make_shared<ProgrammedStop>();
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state =
        std::make_shared<BehaviorRecordingState>();
    std::shared_ptr<MuscleRecordingState> muscle_recording_state =
        std::make_shared<MuscleRecordingState>();

    // Set up cameras acquisition threads
    spdlog::info("Starting behavior camera acquisition thread");
    behavior_recording_state->latest_frame_holder =
        std::make_shared<LatestFrame>();
    std::thread behavior_image_acquirer_thread(
        behavior_image_acquirer,
        recorder_config,
        behavior_recording_state,
        program_state,
        programmed_recording_stop);
    spdlog::info("Behavior camera acquisition thread started");

    spdlog::info("Setting up muscle camera acquisition thread");
    // Image the full sensor of both cameras
    MuscleCameraROI full_frame_roi(
        /*x0=*/1,
        /*x1=*/full_muscle_image_width,
        /*y0=*/1,
        /*y1=*/full_muscle_image_height);
    std::thread muscle_image_acquirer_thread(
        muscle_image_acquirer,
        MuscleCameraROIs{full_frame_roi, full_frame_roi},
        recorder_config,
        profile_dir,
        spdlog::get_level(),
        muscle_recording_state,
        program_state,
        programmed_recording_stop);
    spdlog::info("Muscle camera acquisition thread started");

    // Start Arduino triggering interface set default triggering parameters
    size_t retry_count = 0;
    while (!muscle_recording_state->muscle_camera.load()) {
        spdlog::debug("Waiting for muscle camera to be ready");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retry_count++;
        if (retry_count % 10 == 0) {
            spdlog::warn("Muscle camera is not initialized.");
        }
    }
    int muscle_num_lines_scanned =
        muscle_recording_state->muscle_camera.load()->get_num_lines_scanned();
    arduino_communication = initialize_triggering_with_default_params(
        recorder_config,
        muscle_num_lines_scanned,
        /*sync_ratio=*/1,
        /*muscle_imaging_on=*/true);

    // Show the images until the user saves the ROIs or quits
    AlignCamerasWindow window(
        recorder_config, behavior_recording_state, muscle_recording_state);
    window.show();
    QApplication::exec();

    // Stop the cameras
    spdlog::info("Stopping behavior camera acquisition thread");
    program_state->to_quit.store(true);
    // Give some time for acquisition threads to break out of loop
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (std::shared_ptr<BehaviorCamera> behavior_camera =
            behavior_recording_state->behavior_camera.load()) {
        spdlog::info("Stopping acquisition on behavior camera");
        behavior_camera->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        behavior_recording_state->behavior_camera.store(nullptr);
    }
    muscle_recording_state->muscle_camera.store(nullptr);
    behavior_image_acquirer_thread.join();
    muscle_image_acquirer_thread.join();
    spdlog::info("Behavior camera acquisition thread stopped");

    // Stop triggering. The new protocol has no "stop triggering" command, so
    // switch the blue excitation light off, then close the link.
    spdlog::info("Switching off excitation and closing Arduino link");
    arduino_communication->stop_excitation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arduino_communication->stop_communication();
}

int main(int argc, char **argv) {
    CLIOptions options = parse_cli(argc, argv);
    QApplication application(argc, argv);

    spdlog::set_level(options.log_level);

    // Load recorder configuration
    profile_dir = std::filesystem::path(expand_path(options.profile_dir));

    align_camera(profile_dir);
    spdlog::info("Calibration procedure complete");

    return 0;
}