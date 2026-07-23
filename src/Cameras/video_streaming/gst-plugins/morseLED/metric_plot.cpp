#include "metric_plot.hpp"

#include <algorithm>
#include <cstdio>

namespace {

void plot_set_pixel(std::vector<guint8> &rgb, gint width, gint height, gint x,
                    gint y, guint8 r, guint8 g, guint8 b) {
  if (x < 0 || y < 0 || x >= width || y >= height) {
    return;
  }
  const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) * 3u +
                     static_cast<size_t>(x) * 3u;
  rgb[idx] = r;
  rgb[idx + 1] = g;
  rgb[idx + 2] = b;
}

void plot_draw_line(std::vector<guint8> &rgb, gint width, gint height, gint x0,
                    gint y0, gint x1, gint y1, guint8 r, guint8 g, guint8 b) {
  gint dx = std::abs(x1 - x0);
  gint sx = x0 < x1 ? 1 : -1;
  gint dy = -std::abs(y1 - y0);
  gint sy = y0 < y1 ? 1 : -1;
  gint err = dx + dy;
  while (true) {
    plot_set_pixel(rgb, width, height, x0, y0, r, g, b);
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const gint e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

std::string path_stem(const std::string &path) {
  std::string stem = path;
  const size_t slash = stem.find_last_of("/\\");
  const size_t dot = stem.find_last_of('.');
  if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) {
    stem = stem.substr(0, dot);
  }
  return stem;
}

} // namespace

void MetricPlotRecorder::set_path(const char *path) {
  if (path == nullptr || path[0] == '\0') {
    path_.clear();
    return;
  }
  path_ = path;
}

const gchar *MetricPlotRecorder::path() const {
  return path_.empty() ? nullptr : path_.c_str();
}

void MetricPlotRecorder::append(gdouble timestamp_sec, gfloat metric,
                                gfloat threshold_on, gfloat threshold_off,
                                gboolean led_on) {
  if (!enabled() || history_.size() >= kMaxSamples) {
    return;
  }
  history_.push_back(
      MetricSample{timestamp_sec, metric, threshold_on, threshold_off, led_on});
}

void MetricPlotRecorder::write_and_clear() {
  if (!enabled() || history_.empty()) {
    return;
  }

  const std::string stem = path_stem(path_);
  const std::string csv_path = stem + ".csv";
  const std::string ppm_path = stem + ".ppm";

  FILE *csv = std::fopen(csv_path.c_str(), "w");
  if (csv != nullptr) {
    std::fprintf(csv,
                 "timestamp_sec,metric,threshold_on,threshold_off,led_on\n");
    for (const MetricSample &sample : history_) {
      std::fprintf(csv, "%.6f,%.6f,%.6f,%.6f,%d\n", sample.timestamp_sec,
                   sample.metric, sample.threshold_on, sample.threshold_off,
                   sample.led_on ? 1 : 0);
    }
    std::fclose(csv);
  } else {
    g_warning("metric_plot: failed to write CSV to %s", csv_path.c_str());
  }

  constexpr gint kWidth = 1280;
  constexpr gint kHeight = 480;
  constexpr gint kMarginLeft = 50;
  constexpr gint kMarginRight = 20;
  constexpr gint kMarginTop = 20;
  constexpr gint kMarginBottom = 40;
  const gint plot_w = kWidth - kMarginLeft - kMarginRight;
  const gint plot_h = kHeight - kMarginTop - kMarginBottom;
  if (plot_w <= 1 || plot_h <= 1) {
    history_.clear();
    return;
  }

  std::vector<guint8> rgb(
      static_cast<size_t>(kWidth) * static_cast<size_t>(kHeight) * 3u, 20);
  for (gint y = kMarginTop; y < kHeight - kMarginBottom; ++y) {
    for (gint x = kMarginLeft; x < kWidth - kMarginRight; ++x) {
      plot_set_pixel(rgb, kWidth, kHeight, x, y, 30, 30, 30);
    }
  }
  plot_draw_line(rgb, kWidth, kHeight, kMarginLeft, kMarginTop, kMarginLeft,
                 kHeight - kMarginBottom, 200, 200, 200);
  plot_draw_line(rgb, kWidth, kHeight, kMarginLeft, kHeight - kMarginBottom,
                 kWidth - kMarginRight, kHeight - kMarginBottom, 200, 200, 200);
  {
    const gint y_mid = kMarginTop + static_cast<gint>((1.0 - 0.5) * plot_h);
    plot_draw_line(rgb, kWidth, kHeight, kMarginLeft, y_mid,
                   kWidth - kMarginRight, y_mid, 80, 80, 80);
  }

  const gdouble t0 = history_.front().timestamp_sec;
  const gdouble t1 = history_.back().timestamp_sec;
  const gdouble dt = std::max(1e-6, t1 - t0);

  auto to_x = [&](gdouble t) -> gint {
    return kMarginLeft + static_cast<gint>(((t - t0) / dt) *
                                           static_cast<gdouble>(plot_w - 1));
  };
  auto to_y = [&](gfloat metric) -> gint {
    const gfloat m = std::max(0.0f, std::min(1.0f, metric));
    return kMarginTop +
           static_cast<gint>((1.0f - m) * static_cast<gfloat>(plot_h - 1));
  };

  for (size_t i = 0; i + 1 < history_.size(); ++i) {
    if (!history_[i].led_on) {
      continue;
    }
    const gint x0 = to_x(history_[i].timestamp_sec);
    const gint x1 = to_x(history_[i + 1].timestamp_sec);
    for (gint x = x0; x <= x1; ++x) {
      for (gint y = kMarginTop; y < kHeight - kMarginBottom; ++y) {
        const size_t idx =
            static_cast<size_t>(y) * static_cast<size_t>(kWidth) * 3u +
            static_cast<size_t>(x) * 3u;
        rgb[idx] = static_cast<guint8>(std::min(255, rgb[idx] + 40));
      }
    }
  }

  for (size_t i = 1; i < history_.size(); ++i) {
    const MetricSample &a = history_[i - 1];
    const MetricSample &b = history_[i];
    plot_draw_line(rgb, kWidth, kHeight, to_x(a.timestamp_sec),
                   to_y(a.threshold_on), to_x(b.timestamp_sec),
                   to_y(b.threshold_on), 80, 160, 255);
    plot_draw_line(rgb, kWidth, kHeight, to_x(a.timestamp_sec),
                   to_y(a.threshold_off), to_x(b.timestamp_sec),
                   to_y(b.threshold_off), 255, 180, 60);
  }

  for (size_t i = 1; i < history_.size(); ++i) {
    const MetricSample &a = history_[i - 1];
    const MetricSample &b = history_[i];
    plot_draw_line(rgb, kWidth, kHeight, to_x(a.timestamp_sec), to_y(a.metric),
                   to_x(b.timestamp_sec), to_y(b.metric), 80, 220, 120);
  }

  FILE *ppm = std::fopen(ppm_path.c_str(), "wb");
  if (ppm == nullptr) {
    g_warning("metric_plot: failed to write PPM to %s", ppm_path.c_str());
    history_.clear();
    return;
  }
  std::fprintf(ppm, "P6\n%d %d\n255\n", kWidth, kHeight);
  std::fwrite(rgb.data(), 1, rgb.size(), ppm);
  std::fclose(ppm);

  g_print("morseLED: wrote metric plot %s and CSV %s (%zu samples)\n",
          ppm_path.c_str(), csv_path.c_str(), history_.size());
  history_.clear();
}
