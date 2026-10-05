#pragma once

#include <vector>

#include <QColor>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QWidget>
#include <opencv2/opencv.hpp>

// Qt widgets and helpers shared by the recorder programs.

// Convert an 8-bit grayscale or BGR image to a QImage. A grayscale QImage
// shares the cv::Mat's data, so the cv::Mat must outlive it (or copy it, e.g.
// via QPixmap::fromImage).
QImage cv_mat_to_q_image(const cv::Mat &mat);

// Height of MuscleHistogramWidget (px)
inline constexpr int histogram_widget_height = 65;

// Live histogram of the muscle camera image with a two-handle range slider
// underneath. The two handles select the [vmin, vmax] intensity window used to
// normalize the displayed muscle image (pixels <= vmin are black, >= vmax are
// white). The min handle can never cross past the max handle. Both the
// histogram x-axis and the slider span the fixed [histogram_min, histogram_max]
// intensity range read from the recorder config. The histogram bars are drawn
// in `bin_color`.
class MuscleHistogramWidget : public QWidget {
  public:
    MuscleHistogramWidget(
        int histogram_min,
        int histogram_max,
        int default_vmin,
        int default_vmax,
        QColor bin_color = QColor(180, 180, 180),
        QWidget *parent = nullptr);

    // Recompute the histogram from a 16-bit (CV_16UC1) muscle frame and
    // repaint.
    void set_image(const cv::Mat &image16_bit);
    // set_image(), then return the frame converted to 8 bits with the selected
    // [vmin, vmax] window (empty if the frame is empty).
    cv::Mat update_and_normalize(const cv::Mat &image16_bit);

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
    QColor bin_color_;
    std::vector<float> histogram_; // bin heights normalized to [0, 1]
    DraggedHandle dragged_handle_ = DraggedHandle::none;
};
