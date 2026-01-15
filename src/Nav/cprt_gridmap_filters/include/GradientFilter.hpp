#pragma once

#include <Eigen/Core>
#include <filters/filter_base.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/utilities.hpp>

namespace grid_map {

class GradientFilter : public filters::FilterBase<grid_map::GridMap> {
public:
  GradientFilter() = default;
  virtual ~GradientFilter() = default;
  bool configure() override;
  bool update(const grid_map::GridMap &map_in,
              grid_map::GridMap &map_out) override;
  enum class Stencil { CentralDifference, Sobel };
  enum class Mode { Magnitude, Squared };

private:
  std::string input_layer_ = "elevation";
  std::string output_layer_ = "gradient_mag";

  // "mag" -> sqrt(dzdx^2 + dzdy^2), "sq" -> dzdx^2 + dzdy^2
  Mode mode_ = Mode::Magnitude;

  // "central" uses (x+1 - x-1)/(2*res), "sobel" uses 3x3 sobel stencil
  Stencil stencil_ = Stencil::CentralDifference;

  // If true, will attempt one-sided differences at edges if possible.
  bool allow_one_sided_edges_ = true;

  static constexpr const char *FilterName = "GradientFilter";

  template <typename T>
  bool readParam(grid_map::ParameterReader &param_reader,
                 const std::string &param_name, T &output) {
    if (!param_reader.get(param_name, output)) {
      RCLCPP_ERROR(this->logging_interface_->get_logger(),
                   "%s did not find parameter '%s'.", FilterName,
                   param_name.c_str());
      return false;
    }
    return true;
  }
};

} // namespace grid_map
