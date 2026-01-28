/**
 * BoxBlur.hpp
 *
 *    Created on: Oct 4th, 2025
 *        Author: Lauren Spargo
 *  Organization: Carleton Planetary Robotics Team
 */

#ifndef CPRTGRIDMAPFILTERS_BOXBLURFILTER_HPP_
#define CPRTGRIDMAPFILTERS_BOXBLURFILTER_HPP_
#include <Eigen/Dense>
#include <filters/filter_base.hpp>
#include <string>

#include <string>
#include <vector>

namespace grid_map {

/*!
 * Filter class to find the mean of the values inside a radius.
 */
template <typename T> class BoxBlurFilter : public filters::FilterBase<T> {
public:
  /*!
   * Constructor
   */
  BoxBlurFilter();

  /*!
   * Destructor.
   */
  virtual ~BoxBlurFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  bool configure() override;

  /*!
   * Computes for each value in the input layer the mean of all values in a
   * radius around it Saves this mean in an additional output layer.
   * @param mapIn grid map containing the input layer.
   * @param mapOut grid map containing the layers of the input map and the new
   * layer.
   */
  bool update(const T &mapIn, T &mapOut) override;

  /*!
   * Averages (unweighted) [2*radius + 1] cells at a time along the rows of a
   * gridmap layer
   */
  void HorizontalBoxBlur(const Eigen::MatrixXf &layerIn,
                         Eigen::MatrixXf &layerOut, int r);

  /*!
   * Averages (unweighted) [2*radius + 1] cells at a time along the columns of a
   * gridmap layer, called subsequently after HorizontalBoxBlur() to complete
   * the box blur
   */
  void VerticalBoxBlur(const Eigen::MatrixXf &layerIn,
                       Eigen::MatrixXf &layerOut, int r);

private:
  //! Radius to take the mean from.
  int radius_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

} // namespace grid_map

#endif // CPRTGRIDMAPFILTERS_BOXBLURFILTER_HPP_
