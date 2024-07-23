#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
 
int main(int argc, char** argv) {
  ros::init(argc, argv, "romi_camera");
  ros::NodeHandle nh;

  // Image_transport is used to publish compressed images
  image_transport::ImageTransport it(nh);
  // Publish to the /camera topic
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  // Default /dev/video0 device 
  cv::VideoCapture capture(0);

  // Check that camera can be opened
  if (!capture.isOpened()) return 1;

  sensor_msgs::ImagePtr msg;
  cv::Mat frame; // Mat is the image class defined in OpenCV
  ros::Rate rate(10);

  while (nh.ok()) {
    // Load image
    capture >> frame; 

    // Check if frame has content
    if (!frame.empty()) {
      // Convert image from cv::Mat (OpenCV) type to sensor_msgs/Image (ROS) type and publish
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    rate.sleep();
  }

  // Shutdown camera
  capture.release();
}