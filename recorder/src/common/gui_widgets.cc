#include "recorder/common/gui_widgets.h"

#include <algorithm>
#include <cstdlib>

#include "recorder/common/utils.h"

namespace {
// Layout constants for the muscle histogram + range slider widget.
constexpr int histogram_num_bins = 256;
constexpr int slider_area_height = 12;
constexpr int handle_half_width = 5;
} // namespace

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

MuscleHistogramWidget::MuscleHistogramWidget(
    int histogram_min,
    int histogram_max,
    int default_vmin,
    int default_vmax,
    QColor bin_color,
    QWidget *parent)
    : QWidget(parent), histogram_min_(histogram_min),
      histogram_max_(histogram_max), vmin_(default_vmin), vmax_(default_vmax),
      bin_color_(bin_color), histogram_(histogram_num_bins, 0.0f) {
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

cv::Mat MuscleHistogramWidget::update_and_normalize(const cv::Mat &image16_bit) {
    cv::Mat image8_bit;
    if (image16_bit.empty()) {
        return image8_bit;
    }
    set_image(image16_bit);
    convert16_bit_to8_bit(image16_bit, image8_bit, vmin_, vmax_);
    return image8_bit;
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
    painter.setBrush(bin_color_);
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
