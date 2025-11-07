/**
 * PreserveCostInflationFilter.hpp
 *
 *    Created on: Jan 19, 2025
 *        Author: Erik Caldwell
 *  Organization: Carleton Planetary Robotics Team
 */

#include "PreserveCostInflationFilter.hpp"

#include <cmath>
#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <string>

#include "grid_map_cv/utilities.hpp"
typedef unsigned int uint;

namespace grid_map {

template <typename T>
PreserveCostInflationFilter<T>::PreserveCostInflationFilter()
    : method_(Method::RadialInflationSerial), coreInflationRadius_(0.0),
      decayInflationRadius_(0.0), decayRate_(0.0) {
  ///
}

template <typename T>
PreserveCostInflationFilter<T>::~PreserveCostInflationFilter() {
  ///
}

template <typename T> bool PreserveCostInflationFilter<T>::configure() {
  grid_map::ParameterReader param_reader(this->param_prefix_,
                                         this->params_interface_);

  if (!param_reader.get(std::string("input_layer"), this->inputLayer_)) {
    RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "PreserveCostInflationFilter did not find parameter 'input_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("output_layer"), this->outputLayer_)) {
    RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "PreserveCostInflationFilter did not find parameter 'output_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("core_inflation_radius"),
                        this->coreInflationRadius_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "PreserveCostInflationFilter did not find parameter "
                 "'core_inflation_radius'.");
    return false;
  }

  if (!param_reader.get(std::string("decay_inflation_radius"),
                        this->decayInflationRadius_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "PreserveCostInflationFilter did not find parameter "
                 "'decay_inflation_radius'.");
    return false;
  } else if (this->decayInflationRadius_ < this->coreInflationRadius_) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "PreserveCostInflationFilter parameter "
                 "'decay_inflation_radius' must be "
                 ">= core_inflation_radius.");
    return false;
  }

  if (!param_reader.get(std::string("decay_rate"), this->decayRate_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "PreserveCostInflationFilter did not find parameter "
                 "'decay_rate'.");
    return false;
  } else if (this->decayRate_ < 0.0) {
    RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "PreserveCostInflationFilter parameter 'decay_rate' must be >= 0.0.");
    return false;
  }
  return true;
}

template <typename T>
void PreserveCostInflationFilter<T>::create_lookup_table(const T &mapOut) {

  int maxDx_ = static_cast<int>(std::ceil(decayInflationRadius_) /
                                mapOut.getResolution());
  if (maxDx_ + 1 == decayTable_.size()) {
    return;
  }
  // prepare table (square)
  decayTable_.resize(maxDx_ + 1, std::vector<float>(maxDx_ + 1, 0.0));

  // fill up tables with values
  for (int dx = 0; dx <= maxDx_; ++dx) {
    for (int dy = 0; dy <= maxDx_; ++dy) {
      float distance = std::sqrt(dx * dx + dy * dy) * mapOut.getResolution();
      if (distance <=
          coreInflationRadius_) { // inside inflation radius so no decay
        decayTable_[dx][dy] = 1.0;
      } else if (distance <=
                 decayInflationRadius_) { // outside core but inside decay
        float exponent = -decayRate_ * (distance - coreInflationRadius_);
        decayTable_[dx][dy] = std::exp(exponent);
      } else { // too far for any decay
        decayTable_[dx][dy] = 0.0;
      }
    }
  }
}

template <typename T>
bool PreserveCostInflationFilter<T>::update(const T &mapIn, T &mapOut) {
  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(this->inputLayer_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "Layer %s does not exist. Unable to apply preserve cost "
                 "inflation filter.",
                 this->inputLayer_.c_str());
    return false;
  }

  mapOut.add(this->outputLayer_);

  this->computeWithSimpleSerialMethod(mapIn, mapOut);

  return true;
}

template <typename T>
void PreserveCostInflationFilter<T>::computeWithSimpleSerialMethod(
    const grid_map::GridMap &mapIn, grid_map::GridMap &mapOut) {
  rclcpp::Clock clock;
  const double start = clock.now().seconds();

  const Eigen::MatrixXf &layerIn = mapOut[this->inputLayer_];

  for (grid_map::GridMapIterator iterator(mapIn); !iterator.isPastEnd();
       ++iterator) {
    if (!mapIn.isValid(*iterator, this->inputLayer_)) {
      continue;
    }

    const grid_map::Index index = *iterator;
    grid_map::Position position;
    mapIn.getPosition(
        index, position); // Get position of cell from grid_map::Index of cell

    const float value = layerIn.coeff(index(0), index(1));
    if (!std::isfinite(value)) {
      continue;
    }

    this->radialInflateSerial(mapOut, position, value);
  }

  const double end = clock.now().seconds();
  RCLCPP_DEBUG_THROTTLE(this->logging_interface_->get_logger(), clock, 2.0,
                        "NORMAL COMPUTATION TIME = %f", (end - start));
}

template <typename T>
inline float PreserveCostInflationFilter<T>::getDecay(uint dx, uint dy) const {
  if (dx < dy) {
    std::swap(dx, dy);
  }
  if (dx > maxDx_ || dy > maxDx_) {
    return 0.0;
  }

  return decayTable_[dx][dy];
}

template <typename T>
void PreserveCostInflationFilter<T>::radialInflateSerial(
    grid_map::GridMap &mapOut, const grid_map::Position &position,
    const float value) {
  for (grid_map::CircleIterator iterator(mapOut, position,
                                         this->decayInflationRadius_);
       !iterator.isPastEnd(); ++iterator) {
    auto &cellValue = mapOut.at(this->outputLayer_, *iterator);

    if (!std::isfinite(cellValue)) {
      cellValue = 0.0;
    }

    grid_map::Position cellPosition;
    mapOut.getPosition(*iterator, cellPosition);
    const double distance = (cellPosition - position).norm();
    if (distance <= this->coreInflationRadius_) {
      cellValue = std::max(cellValue, value);
    } else if (distance <= this->decayInflationRadius_) {
      int dx = static_cast<float>(std::abs(cellPosition.x() - position.x()) /
                                  mapOut.getResolution());
      int dy = static_cast<float>(std::abs(cellPosition.y() - position.y()) /
                                  mapOut.getResolution());

      const double decayedValue = value * getDecay(dx, dy);
      cellValue = std::max(cellValue, static_cast<float>(decayedValue));
    }
  }
}

} // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::PreserveCostInflationFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)
