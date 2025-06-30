#include <QApplication>
#include <QMetaType>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "../include/hsv_range_tool/main_window.hpp"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  
  // Register cv::Mat as Qt meta type for signal/slot connections
  qRegisterMetaType<cv::Mat>("cv::Mat");
  
  MainWindow w;
  w.show();
  return a.exec();
}