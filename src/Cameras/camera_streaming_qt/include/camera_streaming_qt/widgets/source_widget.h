/**
 * @file source_widget.h
 * @brief Header file for the SourceWidget class
 * @author Aria Wong
 *
 * This file contains the declaration of the SourceWidget class, which is
 * responsible for containing all of the widgets that allows the user to
 * customize the properties of a source.
 */

#ifndef SOURCE_WIDGET_H
#define SOURCE_WIDGET_H

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <interfaces/msg/video_source.hpp>
#include <string>
#include <vector>

/**
 * @class SourceWidget
 * @brief A class for displaying and customizing the properties of a source.
 *
 * The SourceWidget class provides functionality for displaying and customizing
 * the properties of a source like the name, width, height, origin X and origin
 * Y.
 */
class SourceWidget : public QWidget {
  Q_OBJECT

 public:
  SourceWidget(QWidget* parent = nullptr);
  ~SourceWidget();

  interfaces::msg::VideoSource* get_source() const { return source_; }

  /**
   * @brief Returns get_requesting_source_names
   */
  bool get_requesting_source_names() const;

  /**
   * @brief Uses names to fill name_combo_box
   *
   * @param names List of source names
   */
  void receive_source_names(std::vector<std::string> names);

 signals:
  /**
   * @brief Removes this widget from the preset and deletes itself
   *
   * @param widget The SourceWidget to remove
   */
  void request_remove(SourceWidget* widget);

  /**
   * @brief Gets the source names of all the cameras from CameraClient
   */
  void request_source_names();

 public slots:
  void set_source_name(QString name);
  void set_width(QString width);
  void set_height(QString height);
  void set_origin_x(QString x);
  void set_origin_y(QString y);

  /**
   * @brief Called when clicking on the remove source button to remove this
   * source from the preset
   */
  void remove_source();

  /**
   * @brief Called when clicking the refresh button to get the source names
   */
  void get_source_names();

 private:
  // Source model
  interfaces::msg::VideoSource* source_;

  QVBoxLayout* main_layout_;
  QLabel* source_name_label_;

  // Name UI
  QHBoxLayout* name_layout_;
  QLabel* name_label_;
  QComboBox* name_combo_box_;
  QPushButton* refresh_sources_button_;

  // Size UI
  QHBoxLayout* size_layout_;
  QLabel* width_label_;
  QLineEdit* width_line_edit_;
  QLabel* height_label_;
  QLineEdit* height_line_edit_;

  QIntValidator* size_validator_;

  // Origin UI
  QHBoxLayout* origin_layout_;
  QLabel* origin_x_label_;
  QLineEdit* origin_x_line_edit_;
  QLabel* origin_y_label_;
  QLineEdit* origin_y_line_edit_;

  QIntValidator* origin_validator_;

  QPushButton* remove_button_;

  bool requesting_source_names_;
};

#endif