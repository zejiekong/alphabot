#include <cv_bridge/cv_bridge.h>
#include <detection_msgs/BoundingBoxes.h>
#include <detection_msgs/BoundingBox.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

cv::CascadeClassifier faceCascade;
ros::Publisher coord_pub, img_pub;

void callback(const sensor_msgs::CompressedImageConstPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::vector<cv::Rect> faces;
  cv::Mat grayImage;
  cv::cvtColor(cv_ptr->image, grayImage, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(grayImage, grayImage);
  faceCascade.detectMultiScale(grayImage, faces, 2, 2,
                               0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

  if (not faces.empty()) {
    
    double imageArea = cv_ptr->image.cols * cv_ptr->image.rows;
    double detectionProbability;
    detection_msgs::BoundingBoxes bb_msg;
    // Draw rectangles around detected faces
    for (const auto &face : faces) {
      cv::rectangle(cv_ptr->image, face, cv::Scalar(0, 255, 0), 2);
      detectionProbability = (face.width * face.height) / imageArea;
      detection_msgs::BoundingBox bb;
      bb.probability = detectionProbability;
      bb.xmin = face.x;
      bb.ymin = face.y-face.height;
      bb.xmax = face.x+face.width;
      bb.ymax = face.y;
      bb_msg.bounding_boxes.push_back(bb);
    }
    coord_pub.publish(bb_msg);
  }
    cv_bridge::CvImage cv_image;
    cv_image.image = cv_ptr->image;
    cv_image.encoding="bgr8";
    sensor_msgs::CompressedImagePtr img_msg = cv_image.toCompressedImageMsg();
    img_pub.publish(img_msg);
  //cv::imshow("preview", cv_ptr->image);
  //cv::waitKey(1);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "face_detection_node");
  ros::NodeHandle n;
  faceCascade.load("/home/ubuntu/Desktop/ros_ws/src/alphabot/alphabot_ros/"
                   "alphabot_perception/face_detection/scripts/"
                   "haarcascade_frontalface_default.xml");
  coord_pub = n.advertise<detection_msgs::BoundingBoxes>("/alphabot/face_detection/coord",1);
  img_pub = n.advertise<sensor_msgs::CompressedImage>("/alphabot/face_detection/image/compressed",1);
  ros::Subscriber sub = n.subscribe("/raspicam_node/image/compressed", 1, callback);
  ros::spin();
}
