#include "widgets/main_widget.h"

#include <QDebug>

MainWidget::MainWidget(QWidget* parent) : QWidget(parent) {
  main_layout_ = new QVBoxLayout(this);

  // Setup presets UI
  preset_layout_ = new QHBoxLayout();

  drive_preset_button_ = new QPushButton("Drive");
  preset_layout_->addWidget(drive_preset_button_);
  connect(drive_preset_button_, &QPushButton::clicked, this,
          &MainWidget::send_drive_preset);

  eef_preset_button_ = new QPushButton("EEF");
  preset_layout_->addWidget(eef_preset_button_);
  connect(eef_preset_button_, &QPushButton::clicked, this,
          &MainWidget::send_eef_preset);

  microscope_preset_button_ = new QPushButton("Microscope");
  preset_layout_->addWidget(microscope_preset_button_);
  connect(microscope_preset_button_, &QPushButton::clicked, this,
          &MainWidget::send_microscope_preset);

  belly_preset_button_ = new QPushButton("Belly");
  preset_layout_->addWidget(belly_preset_button_);
  connect(belly_preset_button_, &QPushButton::clicked, this,
          &MainWidget::send_belly_preset);

  drive_eef_preset_button_ = new QPushButton("Drive + EEF");
  preset_layout_->addWidget(drive_eef_preset_button_);
  connect(drive_eef_preset_button_, &QPushButton::clicked, this,
          &MainWidget::send_drive_eef_preset);

  eef_drive_preset_button_ = new QPushButton("EEF + Drive");
  preset_layout_->addWidget(eef_drive_preset_button_);
  connect(eef_drive_preset_button_, &QPushButton::clicked, this,
          &MainWidget::send_eef_drive_preset);

  main_layout_->addLayout(preset_layout_);

  // Setup signal server UI
  signal_server_layout_ = new QHBoxLayout();

  server_ip_line_edit_ = new QLineEdit("ws://localhost:8443");
  signal_server_layout_->addWidget(server_ip_line_edit_);

  server_connect_button_ = new QPushButton("Connect");
  signal_server_layout_->addWidget(server_connect_button_);

  connect(server_ip_line_edit_, &QLineEdit::textChanged, this,
          &MainWidget::set_signal_server_ip);

  main_layout_->addLayout(signal_server_layout_);

  controls_layout_ = new QHBoxLayout();

  // Setup capture image UI
  capture_image_widget_ = new CaptureImageWidget();
  controls_layout_->addWidget(capture_image_widget_, 1);

  // Setup preset UI
  preset_widget_ = new PresetWidget();

  connect(preset_widget_, &PresetWidget::request_source_names, this,
          &MainWidget::get_source_names);

  connect(preset_widget_, &PresetWidget::send_preset, this,
          &MainWidget::receive_preset);

  controls_layout_->addWidget(preset_widget_, 1);

  main_layout_->addLayout(controls_layout_);

  // Make widgets start at top
  main_layout_->setAlignment(Qt::AlignTop);
}

MainWidget::~MainWidget() {}

void MainWidget::receive_source_names(std::vector<std::string> names) {
  if (!preset_widget_) return;

  preset_widget_->receive_source_names(names);
}

void MainWidget::receive_preset(
    std::vector<interfaces::msg::VideoSource> preset) {
  emit send_preset(preset);
}

void MainWidget::set_signal_server_ip(QString ip) {
  signal_server_ip_ = ip;
  qDebug() << "IP: " << signal_server_ip_;
}

void MainWidget::get_source_names() { emit request_source_names(); }

void MainWidget::send_drive_preset() {
  std::vector<interfaces::msg::VideoSource> preset;
  interfaces::msg::VideoSource drive;
  drive.name = "Drive";
  drive.width = 100;
  drive.height = 100;
  preset.push_back(drive);

  emit send_preset(preset);
}

void MainWidget::send_eef_preset() {
  std::vector<interfaces::msg::VideoSource> preset;
  interfaces::msg::VideoSource eef;
  eef.name = "EndEffector";
  eef.width = 100;
  eef.height = 100;
  preset.push_back(eef);

  emit send_preset(preset);
}

void MainWidget::send_microscope_preset() {
  std::vector<interfaces::msg::VideoSource> preset;
  interfaces::msg::VideoSource microscope;
  microscope.name = "Microscope";
  microscope.width = 100;
  microscope.height = 100;
  preset.push_back(microscope);

  emit send_preset(preset);
}

void MainWidget::send_belly_preset() {
  std::vector<interfaces::msg::VideoSource> preset;
  interfaces::msg::VideoSource belly;
  belly.name = "Bottom";
  belly.width = 100;
  belly.height = 100;
  preset.push_back(belly);

  emit send_preset(preset);
}

void MainWidget::send_drive_eef_preset() {
  std::vector<interfaces::msg::VideoSource> preset;

  interfaces::msg::VideoSource drive;
  interfaces::msg::VideoSource eef;

  drive.name = "Drive";
  drive.width = 100;
  drive.height = 100;
  preset.push_back(drive);

  eef.name = "EndEffector";
  eef.width = 30;
  eef.height = 30;
  eef.origin_x = 70;
  eef.origin_y = 0;
  preset.push_back(eef);

  emit send_preset(preset);
}

void MainWidget::send_eef_drive_preset() {
  std::vector<interfaces::msg::VideoSource> preset;

  interfaces::msg::VideoSource eef;
  interfaces::msg::VideoSource drive;

  eef.name = "EndEffector";
  eef.width = 100;
  eef.height = 100;
  preset.push_back(eef);

  drive.name = "Drive";
  drive.width = 30;
  drive.height = 30;
  drive.origin_x = 70;
  drive.origin_y = 0;
  preset.push_back(drive);

  emit send_preset(preset);
}
