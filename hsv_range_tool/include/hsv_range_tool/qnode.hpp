/**
 * @file /include/hsv_range_tool/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef hsv_range_tool_QNODE_HPP_
#define hsv_range_tool_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#endif
#include <QThread>
#include <QPixmap>

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  QPixmap matToQPixmap(const cv::Mat& mat);

Q_SIGNALS:
  void rosShutDown();
  void imageReceived(const cv::Mat& image);
};

#endif /* hsv_range_tool_QNODE_HPP_ */