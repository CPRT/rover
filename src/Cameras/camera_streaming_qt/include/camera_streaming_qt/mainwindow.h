/**
 * @file mainwindow.h
 * @brief Header file for the MainWindow class
 * @author Aria Wong
 *
 * This file contains the declaration of the MainWindow class.
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <interfaces/msg/video_source.hpp>
#include <string>
#include <vector>

#include "camera_client.h"
#include "widgets/main_widget.h"
#include "widgets/source_widget.h"

/**
 * @class MainWindow
 * @brief This class contains the window that holds all of the widgets in the
 * camera client GUI.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(CameraClient* camera_client = nullptr, QWidget* parent = nullptr);
  ~MainWindow();

 signals:
  /**
   * @brief Gets the source names of all the cameras from CameraClient
   */
  void request_source_names();

 private slots:
  /**
   * @brief Gets source names from the CameraClient
   */
  void get_source_names();

  /**
   * @brief Gives the preset to the camera client, so it can start the video
   */
  void receive_preset(std::vector<interfaces::msg::VideoSource> preset);

 private:
  CameraClient* camera_client_;
  MainWidget* main_widget_;
};
#endif  // MAINWINDOW_H
