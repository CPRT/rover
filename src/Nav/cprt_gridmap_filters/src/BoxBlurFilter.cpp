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
  // Add new layers to the elevation map
  mapOut = mapIn;
  T mapOutHorz = mapIn;

  // Check if layer(s) exist
  if (!mapOut.exists(this->inputLayer_) || !mapOutHorz.exists(this->inputLayer_)) 
  {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "Layer %s does not exist. Unable to apply box blur"
                 "filter.",
                 this->inputLayer_.c_str());
    return false;
  }

  mapOut.add(this->outputLayer_);
  mapOutHorz.add(this->outputLayer_);

  HorizontalBoxBlur(mapIn[outputLayer_], mapOutHorz[outputLayer_], radius_);
  VerticalBoxBlur(mapOutHorz[outputLayer_], mapOut[outputLayer_], radius_);
  return true;
}

void HorizontalBoxBlur(
    const Eigen::MatrixXf &layerIn, Eigen::MatrixXf &layerOut,
    double radius)
{
 
  int height = layerOut.rows();
  int width = layerOut.cols();
  int r = (int)radius;

  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      int minCoeff = std::max(0, x - r);
      int maxCoeff = std::min(width - 1, x + r);
      double sum = 0;

      for(int i = minCoeff; i <= maxCoeff; i++)
      {
        sum += layerIn(i, y);
      }
      layerOut(x, y) = sum/(maxCoeff - minCoeff + 1);
    }
  }
}

void VerticalBoxBlur(
    const Eigen::MatrixXf &layerIn, Eigen::MatrixXf &layerOut,
    double radius)
{
  int height = layerOut.rows();
  int width = layerOut.cols();
  int r = (int)radius;

  for(int x = 0; x < width; x++)
  {
    for(int y = 0; y < height; y++)
    {
      int minCoeff = std::max(0, y - r);
      int maxCoeff = std::min(height - 1, y + r);
      double sum = 0;

      for(int i = minCoeff; i <= maxCoeff; i++)
      {
        sum += layerIn(x, i);
      }
      layerOut(x, y) = sum/(maxCoeff - minCoeff + 1);
    }
  }
}

#if 0
{
  template <typename T> void BoxBlurFilter<T>::MatrixMultiply(const T &mapH, const T &mapV, T &mapOut) 
  //wondering if I should only be passing in LAYERS rather than the whole map for more efficiency?? idk enough abt Eigen tbh
  //TODO: ask Connor (or Darren or Erik!) above question ^^
  {
    //Fill in when horizontal algorithm works properly
    mapOut = mapH*mapV;
  }
}
#endif

#if 0
{
  void MatrixMultiply(const Eigen::MatrixXf &layerHorz, const Eigen::MatrixXf &layerVert, Eigen::MatrixXf &layerOut) 
  {
    //layerOut = layerHorz*layerVert;
    //wondering if I should only be passing in LAYERS rather than the whole map for more efficiency?? idk enough abt Eigen tbh
    //TODO: ask Connor (or Darren or Erik!) above question ^^

    layerOut = layerHorz.dot(layerVert);

  }
}
#endif
} // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::BoxBlurFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)
