#pragma once

#include <glib.h>

#include <string>
#include <vector>

struct MetricSample {
  gdouble timestamp_sec = 0.0;
  gfloat metric = 0.0f;
  gfloat threshold_on = 0.0f;
  gfloat threshold_off = 0.0f;
  gboolean led_on = FALSE;
};

// Records ROI metric samples and writes CSV + PPM plots on demand.
class MetricPlotRecorder {
public:
  void set_path(const char *path);
  const gchar *path() const;

  bool enabled() const { return !path_.empty(); }

  void append(gdouble timestamp_sec, gfloat metric, gfloat threshold_on,
              gfloat threshold_off, gboolean led_on);

  // Writes <stem>.csv and <stem>.ppm when path is set and samples exist.
  // Clears history after a successful or attempted write.
  void write_and_clear();

private:
  static constexpr size_t kMaxSamples = 200000;

  std::string path_;
  std::vector<MetricSample> history_;
};
