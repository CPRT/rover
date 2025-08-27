/**
 * PreserveCostInflationFilter.hpp
 *
 *    Created on: Jan 19, 2025
 *        Author: Erik Caldwell
 *  Organization: Carleton Planetary Robotics Team
 */

#ifndef CPRTGRIDMAPFILTERS_PRESERVECOSTINFLATIONFILTER_HPP_
#define CPRTGRIDMAPFILTERS_PRESERVECOSTINFLATIONFILTER_HPP_

#include <Eigen/Core>
#include <filters/filter_base.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <string>
#include <vector>

namespace grid_map {

enum class Method {
  RadialInflationSerial,
};

/**
 * Preserve Cost Inflation filter will inflate the gridmap by a set size
 * to account for the size of the robot. Behaves very similarly to the Nav2
 * inflation layer except it will preserve the cost values of the cells
 * when it inflates.
 */
template <typename T>
class PreserveCostInflationFilter : public filters::FilterBase<T> {
 public:
  /**
   * Constructor
   */
  PreserveCostInflationFilter();

  /**
   * Destructor.
   */
  virtual ~PreserveCostInflationFilter();

  /**
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /**
   * Inflates a gridmap by the predefined inflation radius.
   *
   * @param mapIn GridMap with the different layers to inflate.
   * @param mapOut GridMap with the inflation applied to the layers.
   */
  bool update(const T &mapIn, T &mapOut) override;

  /**
   * Inflates the gripmap by iterating through each cell, then iterating in a
   * circular pattern around the cell.
   *
   * @param layerIn Layer to inflate.
   * @param layerOut Layer with the inflation applied.
   */
  void computeWithSimpleSerialMethod(const grid_map::GridMap &mapIn,
                                     grid_map::GridMap &mapOut);

  void radialInflateSerial(grid_map::GridMap &mapOut,
                           const Eigen::Vector2d &position, const float value);

 private:
  // Input layer name to inflate.
  std::string inputLayer_;

  // Output layer name.
  std::string outputLayer_;

  // Inflation method
  Method method_;

  // Core Inflation radius
  double coreInflationRadius_;

  // Decay inflation radius
  double decayInflationRadius_;

  // Decay rate
  double decayRate_;

};  // class PreserveCostInflationFilter

}  // namespace grid_map

#endif  // CPRTGRIDMAPFILTERS_PRESERVECOSTINFLATIONFILTER_HPP_
