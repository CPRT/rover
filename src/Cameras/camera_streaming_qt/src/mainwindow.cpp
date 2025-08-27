#include "mainwindow.h"

#include <QDebug>
#include <QLabel>

#include "widgets/main_widget.h"

MainWindow::MainWindow(CameraClient* camera_client, QWidget* parent)
    : QMainWindow(parent) {
  camera_client_ = camera_client;

  // Setup main widget
  main_widget_ = new MainWidget(this);

  connect(main_widget_, &MainWidget::request_source_names, this,
          &MainWindow::get_source_names);

  connect(main_widget_, &MainWidget::send_preset, this,
          &MainWindow::receive_preset);

  setCentralWidget(main_widget_);
}

MainWindow::~MainWindow() {}

void MainWindow::get_source_names() {
  if (!camera_client_ || !main_widget_) return;
  std::vector<std::string> names = camera_client_->get_cameras();
  main_widget_->receive_source_names(names);
}

void MainWindow::receive_preset(
    std::vector<interfaces::msg::VideoSource> preset) {
  if (!camera_client_) return;
  camera_client_->start_video(preset.size(), preset);
}
