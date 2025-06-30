/**
 * @file /include/hsv_range_tool/main_window.hpp
 *
 * @brief Qt based gui for hsv_range_tool.
 *
 * @date August 2024
 **/

#ifndef hsv_range_tool_MAIN_WINDOW_H
#define hsv_range_tool_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

private:
  Ui::MainWindowDesign* ui;
  cv::Mat current_image_;
  cv::Mat hsv_image_;
  cv::Mat mask_image_;
  cv::Mat masked_image_;
  
  void closeEvent(QCloseEvent* event);
  void connectSliders();
  void updateHSVImages();
  void updateResultLabels();  // 추가: 결과 라벨 업데이트 함수
  void displayImage(QPushButton* button, const cv::Mat& image);
  QPixmap matToQPixmap(const cv::Mat& mat);

private Q_SLOTS:
  void onImageReceived(const cv::Mat& image);
  void onSliderValueChanged();
};

#endif  // hsv_range_tool_MAIN_WINDOW_H