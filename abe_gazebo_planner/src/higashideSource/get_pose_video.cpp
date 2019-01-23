#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <limits>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Int64.h>
#include <sstream>
#include <string>

#define DEPTH_IMAGE_TOPIC "/camera/aligned_depth_to_color/image_raw"
#define COLOR_IMAGE_TOPIC "/camera/color/image_raw"

using namespace cv;

Mat img, depth_img;
Mat mat_depth_raw, mat_depth;
Size cap_size(1280, 720);
int frame = 0;
char s;
VideoWriter writer("human_pose_RGB.avi", CV_FOURCC('X', 'V', 'I', 'D'), 15, cap_size);

void save_video(void);
void proc(void);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void depthCallback(const sensor_msgs::ImageConstPtr& depth);

/* カラーイメージの取得 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  img = cv_ptr->image;

  namedWindow("RGB_IMG");
  imshow("RGB_IMG", img);
  proc();
}


/* デプスイメージの取得 */
void depthCallback(const sensor_msgs::ImageConstPtr& depth)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //const float f = 561.15;          // 焦点距離
  depth_img = cv_ptr->image;

  namedWindow("DEPTH_IMG");
  imshow("DEPTH_IMG", depth_img);
}

void save_video(void)
{
  std::stringstream ss;
  ss << "depth_img/depth_" << std::setfill('0') << std::setw(5) << frame << ".xml";
  std::cout << "frame: " << std::setfill('0') << std::setw(5) << frame << std::endl;
  writer.write(img);
  FileStorage cvfs(ss.str(), CV_STORAGE_WRITE);
  write(cvfs, "mat", depth_img);
  std::stringstream ss2;
  ss2 << "color_img/color_" << std::setfill('0') << std::setw(5) << frame << ".xml";
  FileStorage cvfs2(ss2.str(), CV_STORAGE_WRITE);
  write(cvfs2, "mat", img);
  frame++;
  char h = waitKey(10);
}

void proc(void)
{
  char c = waitKey(10);
  if(c == 's')
    s = c;
  std::cout << "press s to start record." << std::endl;
  if(s == 's')
    save_video();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "human_pose");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber color_image_sub;
  ros::Subscriber depth_sub;
  color_image_sub = it.subscribe(COLOR_IMAGE_TOPIC, 1, &imageCallback);
  depth_sub = nh.subscribe(DEPTH_IMAGE_TOPIC, 1, &depthCallback);

  ros::spin();

  return 0;
}
