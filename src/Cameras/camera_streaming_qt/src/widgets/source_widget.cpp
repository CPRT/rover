#include "widgets/source_widget.h"

#include <QDebug>

SourceWidget::SourceWidget(QWidget* parent) : QWidget(parent) {
  source_ = new interfaces::msg::VideoSource();

  main_layout_ = new QVBoxLayout(this);

  source_name_label_ = new QLabel("Source");
  main_layout_->addWidget(source_name_label_);

  // Setup name UI
  name_layout_ = new QHBoxLayout();

  name_label_ = new QLabel("Name: ");
  name_layout_->addWidget(name_label_);

  name_combo_box_ = new QComboBox();

  connect(name_combo_box_, &QComboBox::currentTextChanged, this,
          &SourceWidget::set_source_name);

  name_layout_->addWidget(name_combo_box_);

  refresh_sources_button_ = new QPushButton("Refresh");

  connect(refresh_sources_button_, &QPushButton::clicked, this,
          &SourceWidget::get_source_names);

  name_layout_->addWidget(refresh_sources_button_);

  main_layout_->addLayout(name_layout_);

  // Setup size UI
  size_layout_ = new QHBoxLayout();

  size_validator_ = new QIntValidator();
  size_validator_->setBottom(0);

  width_label_ = new QLabel("Width: ");
  size_layout_->addWidget(width_label_);

  width_line_edit_ = new QLineEdit(QString::number(100));
  width_line_edit_->setValidator(size_validator_);

  connect(width_line_edit_, &QLineEdit::textChanged, this,
          &SourceWidget::set_width);

  // TODO: Add default value to config file
  set_width("100");

  size_layout_->addWidget(width_line_edit_);

  height_label_ = new QLabel("Height: ");
  size_layout_->addWidget(height_label_);

  height_line_edit_ = new QLineEdit(QString::number(100));
  height_line_edit_->setValidator(size_validator_);

  connect(height_line_edit_, &QLineEdit::textChanged, this,
          &SourceWidget::set_height);

  set_height("100");

  size_layout_->addWidget(height_line_edit_);

  main_layout_->addLayout(size_layout_);

  // Setup origin UI
  origin_layout_ = new QHBoxLayout();

  origin_validator_ = new QIntValidator();

  origin_x_label_ = new QLabel("Origin X: ");
  origin_layout_->addWidget(origin_x_label_);

  origin_x_line_edit_ = new QLineEdit(QString::number(0));
  origin_x_line_edit_->setValidator(origin_validator_);

  connect(origin_x_line_edit_, &QLineEdit::textChanged, this,
          &SourceWidget::set_origin_x);

  origin_layout_->addWidget(origin_x_line_edit_);

  origin_y_label_ = new QLabel("Origin Y: ");
  origin_layout_->addWidget(origin_y_label_);

  origin_y_line_edit_ = new QLineEdit(QString::number(0));
  origin_y_line_edit_->setValidator(origin_validator_);

  connect(origin_y_line_edit_, &QLineEdit::textChanged, this,
          &SourceWidget::set_origin_y);

  origin_layout_->addWidget(origin_y_line_edit_);

  main_layout_->addLayout(origin_layout_);

  // Setup remove button
  remove_button_ = new QPushButton("Remove Source");

  connect(remove_button_, &QPushButton::clicked, this,
          &SourceWidget::remove_source);

  main_layout_->addWidget(remove_button_);
}

SourceWidget::~SourceWidget() {}

bool SourceWidget::get_requesting_source_names() const {
  return requesting_source_names_;
}

void SourceWidget::receive_source_names(std::vector<std::string> names) {
  name_combo_box_->clear();
  for (int i = 0; i < names.size(); i++) {
    name_combo_box_->addItem(QString::fromStdString(names[i]));
  }
}

void SourceWidget::set_source_name(QString name) {
  if (!source_) return;
  source_->name = name.toStdString();
  qDebug() << "Changed source name to: "
           << QString::fromStdString(source_->name);
}

void SourceWidget::set_width(QString width) {
  if (!source_ || !width.toInt()) return;
  source_->width = width.toInt();
  qDebug() << "Changed source width to: " << source_->width;
}

void SourceWidget::set_height(QString height) {
  if (!source_ || !height.toInt()) return;
  source_->height = height.toInt();
  qDebug() << "Changed source height to: " << source_->height;
}

void SourceWidget::set_origin_x(QString x) {
  if (!source_ || !x.toInt()) return;
  source_->origin_x = x.toInt();
  qDebug() << "Changed source origin x to: " << source_->origin_x;
}

void SourceWidget::set_origin_y(QString y) {
  if (!source_ || !y.toInt()) return;
  source_->origin_y = y.toInt();
  qDebug() << "Changed source origin y to: " << source_->origin_y;
}

void SourceWidget::remove_source() { emit request_remove(this); }

void SourceWidget::get_source_names() {
  requesting_source_names_ = true;
  emit request_source_names();
}