/**
 * PreserveCostInflationFilter.hpp
 *
 *    Created on: Oct 4th, 2025
 *        Author: Lauren Spargo
 *  Organization: Carleton Planetary Robotics Team
 */

#include "BoxBlurFilter.hpp"

#include <Eigen/Dense>

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map {
// Error trapping for invalid parameters
template <typename T> BoxBlurFilter<T>::BoxBlurFilter() : radius_(0.0) {}

template <typename T> BoxBlurFilter<T>::~BoxBlurFilter() {}

template <typename T> bool BoxBlurFilter<T>::configure() {
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("radius"), radius_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "BoxBlur filter did not find parameter `radius`.");
    return false;
  }

  if (radius_ < 0.0) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "BoxBlur filter: Radius must be greater than zero.");
    return false;
  }

  RCLCPP_DEBUG(this->logging_interface_->get_logger(), "Radius = %f.", radius_);

  if (!param_reader.get(std::string("input_layer"), inputLayer_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "BoxBlur filter did not find parameter `input_layer`.");
    return false;
  }

  RCLCPP_DEBUG(this->logging_interface_->get_logger(),
               "BoxBlur input layer is = %s.", inputLayer_.c_str());

  if (!param_reader.get(std::string("output_layer"), outputLayer_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "BoxBlur filter did not find parameter `output_layer`.");
    return false;
  }

  RCLCPP_DEBUG(this->logging_interface_->get_logger(),
               "BoxBlur output_layer = %s.", outputLayer_.c_str());
  return true;
}

template <typename T> bool BoxBlurFilter<T>::update(const T &mapIn, T &mapOut) {
  // Add new layers to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

  HorizontalBoxBlur(mapIn[outputLayer_], mapOut[outputLayer_], radius_);

  //VerticalBoxBlur(mapIn[outputLayer_], mapOut[outputLayer_], radius_);

  double value;

  // First iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd();
       ++iterator) {
    double valueSum = 0.0;
    int counter = 0;
    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Find the mean in a circle around the center
    for (grid_map::CircleIterator submapIterator(mapOut, center, radius_);
         !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, inputLayer_)) {
        continue;
      }
      value = mapOut.at(inputLayer_, *submapIterator);
      valueSum += value;
      counter++;
    }

    if (counter != 0) {
      mapOut.at(outputLayer_, *iterator) = valueSum / counter;
    }
  }

  return true;
}

// Implement SEPARATE (for separable boxblur) horizontal and vertical blur
// functions, call one after the other Don't forget to add to header file when
// finished!
void HorizontalBoxBlur(
    const Eigen::MatrixXf &layerIn, Eigen::MatrixXf &layerOut,
    double radius) // Unsure about parameter types.. ask about const + &!
{
  layerOut = layerIn; // is this necessary?
  float avgDenominator = 2 * radius + 1;

  for (int y = 0; y <= layerOut.cols(); y++) { // Loop through columns
    float minusCoeff;
    float plusCoeff;
    double sum = 0;

    for (int x = 0; x <= layerOut.rows(); x++) { // Loop through rows
      if (x == 0) {
        minusCoeff = 0;
      } else {
        minusCoeff = layerOut(x - 1, y); // layerOut.getCoeff(x - 1, y);
      }

      if (x + 1 <= layerOut.rows()) {
        plusCoeff = layerOut(x + 1, y); // layerOut.getCoeff(x + 1, y);
      } else {
        plusCoeff = 0;
      }

      sum = plusCoeff - minusCoeff + sum;
      layerOut(x, y) = sum / avgDenominator;
    }
  }
}
/*
void VerticalBoxBlur(
    const Eigen::MatrixXf &layerIn, Eigen::MatrixXf &layerOut,
    double radius) // Unsure about parameter types.. ask about const + &!
{
  //Fill in when horizontal algorithm works properly
}

void MatrixMultiply(
    const Eigen::MatrixXf &layerIn, Eigen::MatrixXf &layerOut,
    double radius) // Unsure about parameter types.. ask about const + &!
{
  //Fill in when horizontal algorithm works properly
}
*/

} // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::BoxBlurFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)
