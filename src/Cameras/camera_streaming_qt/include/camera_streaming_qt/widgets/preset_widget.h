/**
 * @file preset_widget.h
 * @brief Header file for the PresetWidget class.
 * @author Aria Wong
 *
 * This file contains the declaration of the PresetWidget class, which
 * is holds all of the widgets that lets a user customize a preset.
 */

#ifndef PRESET_WIDGET_H
#define PRESET_WIDGET_H

#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>
#include <interfaces/msg/video_source.hpp>
#include <string>
#include <vector>

#include "widgets/source_widget.h"

/**
 * @class PresetWidget
 * @brief A class that holds the widgets to allow a user to customize a preset.
 *
 * The PresetWidget class holds the widgets that let the user customize a
 * preset, mainly the list of sources.
 */
class PresetWidget : public QWidget {
  Q_OBJECT

 public:
  PresetWidget(QWidget* parent = nullptr);
  ~PresetWidget();

 signals:
  /**
   * @brief Gets the source names from CameraClient
   */
  void request_source_names();

  /**
   * @brief Sends this preset to the CameraClient to start video
   */
  void send_preset(std::vector<interfaces::msg::VideoSource> preset);

 public slots:
  /**
   * @brief Adds a new SourceWidget to the preset
   */
  void add_source();

  /**
   * @brief Called when clicking on the submit preset button to send this preset
   * to the CameraClient.
   */
  void submit_preset();

  /**
   * @brief Removes given SourceWidget from this preset and deletes it
   */
  void remove_source(SourceWidget* src);

  /**
   * @brief Gets source names from CameraClient
   */
  void receive_source_names(std::vector<std::string> sources);

 private slots:
  /**
   * @brief Called when clicking the refresh button in SourceWidget to get the
   * source names
   */
  void get_source_names();

 private:
  std::vector<SourceWidget*> sources_;

  QLabel* widget_header_;

  QWidget* sources_container_;
  QVBoxLayout* sources_layout_;
  QScrollArea* sources_scroll_area_;
  QVBoxLayout* main_layout_;

  QPushButton* add_source_button_;
  QPushButton* submit_preset_button_;
};

#endif