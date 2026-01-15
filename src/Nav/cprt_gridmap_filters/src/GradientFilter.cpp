#include "GradientFilter.hpp"

#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace grid_map {

static inline bool isFinite(float value) { return std::isfinite(value); }
static inline float quietNaN() {
  return std::numeric_limits<float>::quiet_NaN();
}

bool GradientFilter::configure() {
  grid_map::ParameterReader param_reader(this->param_prefix_,
                                         this->params_interface_);
  if (!readParam<std::string>(param_reader, "input_layer", input_layer_)) {
    return false;
  }
  if (!readParam<std::string>(param_reader, "output_layer", output_layer_)) {
    return false;
  }
  std::string stencil_str;
  if (!readParam<std::string>(param_reader, "stencil", stencil_str)) {
    return false;
  }
  if (stencil_str == "central" || stencil_str == "cen") {
    stencil_ = Stencil::CentralDifference;
  } else if (stencil_str == "sob" || stencil_str == "sobel") {
    stencil_ = Stencil::Sobel;
  } else {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "%s parameter 'stencil' has invalid value '%s'. Options are: "
                 "central (cen) or sobel (sob).",
                 FilterName, stencil_str.c_str());
    return false;
  }

  std::string mode_str;
  if (!readParam<std::string>(param_reader, "mode", mode_str)) {
    return false;
  }
  if (mode_str == "mag" || mode_str == "magnitude") {
    mode_ = Mode::Magnitude;
  } else if (mode_str == "sq" || mode_str == "squared") {
    mode_ = Mode::Squared;
  } else {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "%s parameter 'mode' has invalid value '%s'. Options are: "
                 "magnitude (mag) or squared (sq).",
                 FilterName, mode_str.c_str());
    return false;
  }
  if (!readParam<bool>(param_reader, "allow_one_sided_edges",
                       allow_one_sided_edges_)) {
    return false;
  }
  return true;
}

static inline bool in_bounds(int row, int col, int num_rows, int num_cols) {
  return (row >= 0 && row < num_rows && col >= 0 && col < num_cols);
}

bool GradientFilter::update(const grid_map::GridMap &map_in,
                            grid_map::GridMap &map_out) {
  map_out = map_in;
  if (!map_out.exists(output_layer_)) {
    map_out.add(output_layer_);
  }

  const auto &height_layer = map_in[input_layer_];
  auto &gradient_layer = map_out[output_layer_];

  const int num_rows = height_layer.rows();
  const int num_cols = height_layer.cols();
  const float cell_size_m = static_cast<float>(map_in.getResolution());

  for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {

      const float center_height = height_layer(row, col);
      if (!isFinite(center_height)) {
        gradient_layer(row, col) = quietNaN();
        continue;
      }

      float dHeight_dx = quietNaN();
      float dHeight_dy = quietNaN();

      if (stencil_ == Stencil::CentralDifference) {
        // ---- central difference ----
        const int col_left = col - 1;
        const int col_right = col + 1;
        const int row_up = row - 1;
        const int row_down = row + 1;

        const bool has_left = in_bounds(row, col_left, num_rows, num_cols) &&
                              isFinite(height_layer(row, col_left));
        const bool has_right = in_bounds(row, col_right, num_rows, num_cols) &&
                               isFinite(height_layer(row, col_right));
        const bool has_up = in_bounds(row_up, col, num_rows, num_cols) &&
                            isFinite(height_layer(row_up, col));
        const bool has_down = in_bounds(row_down, col, num_rows, num_cols) &&
                              isFinite(height_layer(row_down, col));

        // x derivative
        if (has_left && has_right) {
          dHeight_dx =
              (height_layer(row, col_right) - height_layer(row, col_left)) /
              (2.0f * cell_size_m);
        } else if (allow_one_sided_edges_) {
          if (has_right) {
            dHeight_dx =
                (height_layer(row, col_right) - center_height) / cell_size_m;
          } else if (has_left) {
            dHeight_dx =
                (center_height - height_layer(row, col_left)) / cell_size_m;
          }
        }

        // y derivative
        if (has_up && has_down) {
          dHeight_dy =
              (height_layer(row_down, col) - height_layer(row_up, col)) /
              (2.0f * cell_size_m);
        } else if (allow_one_sided_edges_) {
          if (has_down) {
            dHeight_dy =
                (height_layer(row_down, col) - center_height) / cell_size_m;
          } else if (has_up) {
            dHeight_dy =
                (center_height - height_layer(row_up, col)) / cell_size_m;
          }
        }

      } else {
        // ---- 3x3 Sobel ----
        // Only valid for interior cells
        const bool is_border_cell = (row == 0 || row == num_rows - 1 ||
                                     col == 0 || col == num_cols - 1);
        if (is_border_cell) {
          gradient_layer(row, col) = quietNaN();
          continue;
        }

        const float h_ul = height_layer(row - 1, col - 1);
        const float h_uc = height_layer(row - 1, col);
        const float h_ur = height_layer(row - 1, col + 1);
        const float h_ml = height_layer(row, col - 1);
        const float h_mr = height_layer(row, col + 1);
        const float h_ll = height_layer(row + 1, col - 1);
        const float h_lc = height_layer(row + 1, col);
        const float h_lr = height_layer(row + 1, col + 1);

        const bool all_neighbors_finite =
            (isFinite(h_ul) && isFinite(h_uc) && isFinite(h_ur) &&
             isFinite(h_ml) && isFinite(h_mr) && isFinite(h_ll) &&
             isFinite(h_lc) && isFinite(h_lr));

        if (!all_neighbors_finite) {
          gradient_layer(row, col) = quietNaN();
          continue;
        }

        const float sobel_gx = (-1.f * h_ul + 1.f * h_ur) +
                               (-2.f * h_ml + 2.f * h_mr) +
                               (-1.f * h_ll + 1.f * h_lr);

        const float sobel_gy = (-1.f * h_ul - 2.f * h_uc - 1.f * h_ur) +
                               (1.f * h_ll + 2.f * h_lc + 1.f * h_lr);

        dHeight_dx = sobel_gx / (8.0f * cell_size_m);
        dHeight_dy = sobel_gy / (8.0f * cell_size_m);
      }

      if (!isFinite(dHeight_dx) || !isFinite(dHeight_dy)) {
        gradient_layer(row, col) = quietNaN();
        continue;
      }

      const float grad_sq = dHeight_dx * dHeight_dx + dHeight_dy * dHeight_dy;
      gradient_layer(row, col) =
          (mode_ == Mode::Squared) ? grad_sq : std::sqrt(grad_sq);
    }
  }

  return true;
}

} // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::GradientFilter,
                       filters::FilterBase<grid_map::GridMap>)
