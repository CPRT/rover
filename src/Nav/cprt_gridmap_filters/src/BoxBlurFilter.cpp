/**
 * BoxBlurFilter.hpp
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
//Error trapping for invalid parameters
template <typename T> BoxBlurFilter<T>::BoxBlurFilter() : radius_(0.0) {}

template <typename T> BoxBlurFilter<T>::~BoxBlurFilter() {}

template <typename T> bool BoxBlurFilter<T>::configure() {
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("radius"), radius_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "BoxBlur filter did not find parameter `radius`.");
    return false;
  }

  if (radius_ < 0) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "BoxBlur filter: Radius must be greater than zero.");
    return false;
  }

  RCLCPP_DEBUG(this->logging_interface_->get_logger(), "Radius = %d.", radius_);

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
  mapOut = mapIn;
  T mapOutHorz = mapIn;

  // Check if layer(s) exist
  if (!mapOut.exists(this->inputLayer_) ||
      !mapOutHorz.exists(this->inputLayer_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "Layer %s does not exist. Unable to apply box blur"
                 "filter.",
                 this->inputLayer_.c_str());
    return false;
  }

  mapOut.add(this->outputLayer_);
  mapOutHorz.add(this->outputLayer_);

  HorizontalBoxBlur(mapIn[inputLayer_], mapOutHorz[outputLayer_], radius_); //Blurs input layer from mapIn horizontally, saves blurred values to mapOutHorz on outputLayer_
  VerticalBoxBlur(mapOutHorz[outputLayer_], mapOut[outputLayer_], radius_); //Blurs outputLayer_ from mapOutHorz vertically, saves blurred values to mapOut on outputLayer_
  //Box blur complete!
  return true;
}

template <typename T>
void BoxBlurFilter<T>::HorizontalBoxBlur(const Eigen::MatrixXf &layerIn, Eigen::MatrixXf &layerOut, int r) 
{
  int height = layerOut.rows();
  int width = layerOut.cols();

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      //ensuring that max and min cells in radius do not exceed dimensions of gridmap layer
      int minCoeff = std::max(0, x - r);
      int maxCoeff = std::min(width - 1, x + r);
      int numHoles = 0;
      double sum = 0;

      for (int i = minCoeff; i <= maxCoeff; i++) {
        const float value = layerIn(i, y);
        if (!std::isfinite(value)) //checking for hole
        {
          numHoles += 1; //if hole found, disregard it and continue
          continue;
        }
        sum += value;
      }
      //calculate and write average to layerOut
      layerOut(x, y) = sum / (maxCoeff - minCoeff + 1 - numHoles);
    }
  }
}

template <typename T>
void BoxBlurFilter<T>::VerticalBoxBlur(const Eigen::MatrixXf &layerIn, Eigen::MatrixXf &layerOut, int r) 
{
  //Same algorithm as horizontal box blur, only along a column rather than a row
  int height = layerOut.rows();
  int width = layerOut.cols();

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      int minCoeff = std::max(0, y - r);
      int maxCoeff = std::min(height - 1, y + r);
      int numHoles = 0;
      double sum = 0;

      for (int i = minCoeff; i <= maxCoeff; i++) {
        const float value = layerIn(x, i);
        if (!std::isfinite(value)) {
          numHoles += 1;
          continue;
        }
        sum += value;
      }
      layerOut(x, y) = sum / (maxCoeff - minCoeff + 1 - numHoles);
    }
  }
}

} // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::BoxBlurFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)
