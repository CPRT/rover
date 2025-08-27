/**
 * @file main_widget.h
 * @brief Header file for the MainWidget class
 * @author Aria Wong
 *
 * This file contains the declaration of the MainWidget class, which is
 * responsible for displaying all of the widgets in the camera streaming
 * application.
 */

#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <interfaces/msg/video_source.hpp>
#include <string>
#include <vector>

#include "widgets/capture_image_widget.h"
#include "widgets/preset_widget.h"
#include "widgets/source_widget.h"

/**
 * @class MainWidget
 * @brief A class for displaying all of the widgets in the camera streaming
 * application.
 *
 * The MainWidget class combines all of the smaller widgets required to use the
 * camera streaming application and displays them.
 */
class MainWidget : public QWidget {
  Q_OBJECT

 public:
  MainWidget(QWidget* parent = nullptr);
  ~MainWidget();

  void receive_source_names(std::vector<std::string> names);

 signals:
  void request_source_names();
  void send_preset(std::vector<interfaces::msg::VideoSource> preset);

 public slots:
  /**
   * @brief Slot that sets the signal server IP that gets called when the
   * server_ip_line_edit_ gets modified.
   *
   * @param ip IP address for the signal server
   */
  void set_signal_server_ip(QString ip);

  /**
   * @brief Receives preset to send to CameraClient
   */
  void receive_preset(std::vector<interfaces::msg::VideoSource> preset);

 private slots:
  /**
   * @brief Called when clicking the refresh button in SourceWidget to get the
   * source names
   */
  void get_source_names();

 private:
  QVBoxLayout* main_layout_;

  // Presets UI
  QHBoxLayout* preset_layout_;

  QPushButton* drive_preset_button_;
  QPushButton* eef_preset_button_;
  QPushButton* microscope_preset_button_;
  QPushButton* belly_preset_button_;
  QPushButton* drive_eef_preset_button_;
  QPushButton* eef_drive_preset_button_;

  // Functions for sending hardcoded presets to CameraClient
  void send_drive_preset();
  void send_eef_preset();
  void send_microscope_preset();
  void send_belly_preset();
  void send_drive_eef_preset();
  void send_eef_drive_preset();

  // Signal server UI
  QHBoxLayout* signal_server_layout_;
  QLineEdit* server_ip_line_edit_;
  QPushButton* server_connect_button_;

  QString signal_server_ip_;

  // Layout for horizontally displaying capture image and preset UIs
  QHBoxLayout* controls_layout_;

  // Capture Image UI
  CaptureImageWidget* capture_image_widget_;

  // Preset UI
  PresetWidget* preset_widget_;
};

#endif