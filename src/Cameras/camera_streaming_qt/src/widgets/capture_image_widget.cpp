#include "widgets/capture_image_widget.h"

#include <QDebug>

CaptureImageWidget::CaptureImageWidget(QWidget* parent) : QWidget(parent) {
  main_layout_ = new QVBoxLayout(this);

  widget_header_ = new QLabel("Capture Image");
  main_layout_->addWidget(widget_header_);

  source_layout_ = new QHBoxLayout();

  // Setup source name and refresh button
  source_name_label_ = new QLabel("Name: ");
  source_layout_->addWidget(source_name_label_);

  source_name_combo_box_ = new QComboBox();
  connect(source_name_combo_box_, &QComboBox::currentTextChanged, this,
          &CaptureImageWidget::set_source_name);
  source_layout_->addWidget(source_name_combo_box_);

  refresh_button_ = new QPushButton("Refresh");
  source_layout_->addWidget(refresh_button_);

  main_layout_->addLayout(source_layout_);

  // Setup filename
  filename_layout_ = new QHBoxLayout();

  filename_label_ = new QLabel("Filename (Optional): ");
  filename_layout_->addWidget(filename_label_);

  filename_line_edit_ = new QLineEdit();
  connect(filename_line_edit_, &QLineEdit::textChanged, this,
          &CaptureImageWidget::set_filename);
  filename_layout_->addWidget(filename_line_edit_);

  main_layout_->addLayout(filename_layout_);

  capture_image_button_ = new QPushButton("Capture Image");
  main_layout_->addWidget(capture_image_button_);

  main_layout_->setAlignment(Qt::AlignTop);
}

CaptureImageWidget::~CaptureImageWidget() {}

void CaptureImageWidget::set_source_name(QString name) {
  source_name_ = name;
  qDebug() << "Changed capture image source to: " << name;
}

void CaptureImageWidget::set_filename(QString filename) {
  filename_ = filename;
  qDebug() << "Changed capture image filename to: " << filename;
}