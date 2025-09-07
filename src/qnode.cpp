/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/hsv_range_tool/qnode.hpp"
#include <QImage>

QNode::QNode()
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("hsv_range_tool");
  
  image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/path_rs/path_rs/color/image_raw", 
    10, 
    std::bind(&QNode::imageCallback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(node->get_logger(), "HSV Range Tool node started.");
  
  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(30);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
    
    QThread::msleep(1);
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    // ROS image를 OpenCV Mat으로 변환
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    
    // 이미지 신호 방출
    Q_EMIT imageReceived(image);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

QPixmap QNode::matToQPixmap(const cv::Mat& mat)
{
  switch (mat.type())
  {
    case CV_8UC4:
    {
      QImage qimg(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
      return QPixmap::fromImage(qimg);
    }
    case CV_8UC3:
    {
      QImage qimg(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
      return QPixmap::fromImage(qimg.rgbSwapped());
    }
    case CV_8UC1:
    {
      QImage qimg(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
      return QPixmap::fromImage(qimg);
    }
  }
  return QPixmap();
}