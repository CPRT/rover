/**
 * @file capture_image_widget.h
 * @brief Header file for the CaptureImageWidget class
 * @author Aria Wong
 *
 * This file contains the declaration of the CaptureImageWidget
 * class, which holds the widgets that allow the user to take
 * an image of one of the rover sources.
 */

#ifndef CAPTURE_IMAGE_WIDGET_H
#define CAPTURE_IMAGE_WIDGET_H

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

/**
 * @class CaptureImageWidget
 * @brief A class that holds the widgets, allowing the user to capture an image.
 *
 * The CaptureImageWidget class provides the widgets that allow the user to take
 * an image of the video sources on the rover.
 */
class CaptureImageWidget : public QWidget {
  Q_OBJECT

 public:
  CaptureImageWidget(QWidget* parent = nullptr);
  ~CaptureImageWidget();

 public slots:
  void set_source_name(QString name);
  void set_filename(QString filename);

 private:
  QVBoxLayout* main_layout_;

  QLabel* widget_header_;

  // Source name and refresh button
  QString source_name_;
  QHBoxLayout* source_layout_;
  QLabel* source_name_label_;
  QComboBox* source_name_combo_box_;
  QPushButton* refresh_button_;

  // Filename
  QString filename_;
  QHBoxLayout* filename_layout_;
  QLabel* filename_label_;
  QLineEdit* filename_line_edit_;

  QPushButton* capture_image_button_;
};

#endif