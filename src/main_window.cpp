/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/hsv_range_tool/main_window.hpp"
#include <QImage>
#include <iostream>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);
  std::cout << "Step 1: UI setup complete" << std::endl;

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);
  
  // 창 크기와 위치 설정
  this->resize(2000, 1000);
  this->move(100, 100);
  std::cout << "Step 2: Window setup complete" << std::endl;

  // Show window first and bring to front
  this->show();
  this->raise();
  this->activateWindow();
  this->setWindowState(Qt::WindowActive);
  std::cout << "Step 3: Window shown" << std::endl;

  // Setup sliders and spinboxes FIRST
  connectSliders();
  std::cout << "Step 4: Sliders connected" << std::endl;
  
  // Set initial HSV ranges (full range)
  ui->H_min->setValue(0);
  ui->H_max->setValue(179);
  ui->S_min->setValue(0);
  ui->S_max->setValue(255);
  ui->V_min->setValue(0);
  ui->V_max->setValue(255);
  std::cout << "Step 5: Initial values set" << std::endl;
  
  // Initialize result labels AFTER sliders are set
  updateResultLabels();
  std::cout << "Step 6: Result labels updated" << std::endl;

  // Create QNode but don't start yet
  qnode = new QNode();
  std::cout << "Step 7: QNode created" << std::endl;

  // Connect ROS signals with QUEUED connection for thread safety
  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()), Qt::QueuedConnection);
  QObject::connect(qnode, SIGNAL(imageReceived(const cv::Mat&)), 
                   this, SLOT(onImageReceived(const cv::Mat&)), Qt::QueuedConnection);
  std::cout << "Step 8: Signals connected" << std::endl;
  
  // Debug output
  std::cout << "=== HSV Range Tool GUI initialized ===" << std::endl;
  std::cout << "Window size: " << this->width() << "x" << this->height() << std::endl;
  std::cout << "Window visible: " << this->isVisible() << std::endl;
  std::cout << "Waiting for camera images..." << std::endl;
}

void MainWindow::connectSliders()
{
  // Connect H sliders and spinboxes
  connect(ui->H_min, &QSlider::valueChanged, ui->H_min_box, &QSpinBox::setValue);
  connect(ui->H_min_box, QOverload<int>::of(&QSpinBox::valueChanged), ui->H_min, &QSlider::setValue);
  connect(ui->H_max, &QSlider::valueChanged, ui->H_max_box, &QSpinBox::setValue);
  connect(ui->H_max_box, QOverload<int>::of(&QSpinBox::valueChanged), ui->H_max, &QSlider::setValue);
  
  // Connect S sliders and spinboxes
  connect(ui->S_min, &QSlider::valueChanged, ui->S_min_box, &QSpinBox::setValue);
  connect(ui->S_min_box, QOverload<int>::of(&QSpinBox::valueChanged), ui->S_min, &QSlider::setValue);
  connect(ui->S_max, &QSlider::valueChanged, ui->S_max_box, &QSpinBox::setValue);
  connect(ui->S_max_box, QOverload<int>::of(&QSpinBox::valueChanged), ui->S_max, &QSlider::setValue);
  
  // Connect V sliders and spinboxes
  connect(ui->V_min, &QSlider::valueChanged, ui->V_min_box, &QSpinBox::setValue);
  connect(ui->V_min_box, QOverload<int>::of(&QSpinBox::valueChanged), ui->V_min, &QSlider::setValue);
  connect(ui->V_max, &QSlider::valueChanged, ui->V_max_box, &QSpinBox::setValue);
  connect(ui->V_max_box, QOverload<int>::of(&QSpinBox::valueChanged), ui->V_max, &QSlider::setValue);
  
  // Connect all sliders to update function
  connect(ui->H_min, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged);
  connect(ui->H_max, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged);
  connect(ui->S_min, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged);
  connect(ui->S_max, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged);
  connect(ui->V_min, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged);
  connect(ui->V_max, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged);
}

void MainWindow::onImageReceived(const cv::Mat& image)
{
  current_image_ = image.clone();
  
  // Debug output (매 30프레임마다만 출력)
  static int frame_count = 0;
  if (++frame_count % 30 == 0)
  {
    std::cout << "Image received: " << image.cols << "x" << image.rows 
              << " (Frame #" << frame_count << ")" << std::endl;
  }
  
  // Display original image
  displayImage(ui->original_image, current_image_);
  
  // Update HSV processing
  updateHSVImages();
}

void MainWindow::onSliderValueChanged()
{
  updateResultLabels();  // 결과 라벨 업데이트
  
  if (!current_image_.empty())
  {
    updateHSVImages();
  }
}

void MainWindow::updateHSVImages()
{
  if (current_image_.empty()) return;
  
  // Convert to HSV (PathFind와 동일)
  cv::cvtColor(current_image_, hsv_image_, cv::COLOR_BGR2HSV);
  
  // Apply median blur (PathFind의 enhancedHSVFilter와 동일)
  cv::medianBlur(hsv_image_, hsv_image_, 5);
  
  // Get HSV ranges from sliders
  int h_min = ui->H_min->value();
  int h_max = ui->H_max->value();
  int s_min = ui->S_min->value();
  int s_max = ui->S_max->value();
  int v_min = ui->V_min->value();
  int v_max = ui->V_max->value();
  
  // Create HSV color spectrum image (기존 유지)
  cv::Mat hsv_spectrum = cv::Mat::zeros(current_image_.size(), CV_8UC3);
  
  for (int y = 0; y < hsv_spectrum.rows; y++)
  {
    for (int x = 0; x < hsv_spectrum.cols; x++)
    {
      int h_val = h_min + (h_max - h_min) * x / hsv_spectrum.cols;
      int s_val = s_max - (s_max - s_min) * y / hsv_spectrum.rows;
      int v_val = (v_min + v_max) / 2;
      
      hsv_spectrum.at<cv::Vec3b>(y, x) = cv::Vec3b(h_val, s_val, v_val);
    }
  }
  
  cv::Mat hsv_color_display;
  cv::cvtColor(hsv_spectrum, hsv_color_display, cv::COLOR_HSV2BGR);
  displayImage(ui->hsv_color, hsv_color_display);
  
  // Enhanced HSV filtering (PathFind의 enhancedHSVFilter 로직 적용)
  if (h_min > h_max) {
    // 색상이 180도를 넘어가는 경우 (예: 빨간색)
    cv::Mat mask1, mask2;
    
    // 첫 번째 범위
    cv::Scalar lower_bound1(h_min, s_min, v_min);
    cv::Scalar upper_bound1(179, s_max, v_max);
    cv::inRange(hsv_image_, lower_bound1, upper_bound1, mask1);
    
    // 두 번째 범위
    cv::Scalar lower_bound2(0, s_min, v_min);
    cv::Scalar upper_bound2(h_max, s_max, v_max);
    cv::inRange(hsv_image_, lower_bound2, upper_bound2, mask2);
    
    // 두 마스크 합치기
    mask_image_ = mask1 + mask2;
  } else {
    // 일반 색상 영역
    cv::Scalar lower_bound(h_min, s_min, v_min);
    cv::Scalar upper_bound(h_max, s_max, v_max);
    cv::inRange(hsv_image_, lower_bound, upper_bound, mask_image_);
  }
  
  // Apply morphological operations (PathFind와 동일)
  static cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(-1, -1));
  cv::morphologyEx(mask_image_, mask_image_, cv::MORPH_OPEN, element);
  cv::morphologyEx(mask_image_, mask_image_, cv::MORPH_CLOSE, element);
  
  // Display white mask
  displayImage(ui->masked_white, mask_image_);
  
  // Apply mask to original image
  masked_image_ = cv::Mat::zeros(current_image_.size(), current_image_.type());
  current_image_.copyTo(masked_image_, mask_image_);
  displayImage(ui->masked_raw, masked_image_);
}

void MainWindow::updateResultLabels()
{
  // Get current slider values
  int h_min = ui->H_min->value();
  int h_max = ui->H_max->value();
  int s_min = ui->S_min->value();
  int s_max = ui->S_max->value();
  int v_min = ui->V_min->value();
  int v_max = ui->V_max->value();
  
  // Update Upper bound labels
  ui->H_result_upper->setText(QString::number(h_max));
  ui->S_result_upper->setText(QString::number(s_max));
  ui->V_result_upper->setText(QString::number(v_max));
  
  // Update Lower bound labels  
  ui->H_result_lower->setText(QString::number(h_min));
  ui->S_result_lower->setText(QString::number(s_min));
  ui->V_result_lower->setText(QString::number(v_min));
}

void MainWindow::displayImage(QPushButton* button, const cv::Mat& image)
{
  if (image.empty()) 
  {
    static bool empty_warning_shown = false;
    if (!empty_warning_shown)
    {
      std::cout << "Warning: Empty image received for display" << std::endl;
      empty_warning_shown = true;
    }
    return;
  }
  
  QPixmap pixmap = matToQPixmap(image);
  if (pixmap.isNull())
  {
    static bool pixmap_warning_shown = false;
    if (!pixmap_warning_shown)
    {
      std::cout << "Error: Failed to convert Mat to QPixmap" << std::endl;
      pixmap_warning_shown = true;
    }
    return;
  }
  
  // Scale pixmap to fit button size while maintaining aspect ratio
  QPixmap scaled_pixmap = pixmap.scaled(button->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
  
  // Set pixmap as button icon
  button->setIcon(QIcon(scaled_pixmap));
  button->setIconSize(button->size());
  button->setText("");  // Remove text
}

QPixmap MainWindow::matToQPixmap(const cv::Mat& mat)
{
  switch (mat.type())
  {
    // 8-bit, 4 channel
    case CV_8UC4:
    {
      QImage qimg(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
      return QPixmap::fromImage(qimg);
    }
    // 8-bit, 3 channel
    case CV_8UC3:
    {
      QImage qimg(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
      return QPixmap::fromImage(qimg.rgbSwapped());
    }
    // 8-bit, 1 channel
    case CV_8UC1:
    {
      QImage qimg(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
      return QPixmap::fromImage(qimg);
    }
  }
  return QPixmap();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}