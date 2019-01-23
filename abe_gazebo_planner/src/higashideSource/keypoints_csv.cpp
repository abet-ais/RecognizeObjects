#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <time.h>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <limits>
#include <algorithm>
#include <iterator> 
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#define CX 319.5
#define CY 239.5
#define FX 570.3422241210938
#define FY 570.3422241210938

tf::TransformListener* listener_ptr;
tf2_ros::Buffer* tf_buffer;

void get3dPosition(cv::Mat &depth, std::vector<double> &wrist, std::vector<double> &lhand, std::vector<double> &rhand, int frame);
cv::Mat readDepthImg(int frame);

void cameraPixelTrans(std::vector<cv::Point>& rhand_vec_cam_coords, std::vector<float>& rhand_depth_vec_cam_coords){
  std::vector<float> rhand_cam_x;
  std::vector<float> rhand_cam_y;

  for(int i=0; i<20; i++){
    // std::cout << rhand_vec_cam_coords.at(i).x << ", " << rhand_vec_cam_coords.at(i).y << ", " << rhand_depth_vec_cam_coords.at(i) << std::endl;

    float tmp_rhand_cam_x;
    float tmp_rhand_cam_y;
    float tmp_rhand_depth_cam;

    tmp_rhand_cam_x = (rhand_depth_vec_cam_coords.at(i)*(CX-rhand_vec_cam_coords.at(i).x))/FX;
    tmp_rhand_cam_y = (rhand_depth_vec_cam_coords.at(i)*(CY-rhand_vec_cam_coords.at(i).y))/FY;

    rhand_cam_x.push_back(tmp_rhand_cam_x);
    rhand_cam_y.push_back(tmp_rhand_cam_y);

    // std::cout << rhand_cam_x.at(i) << ", " << rhand_cam_y.at(i) << ", " << rhand_depth_vec_cam_coords.at(i) << std::endl;
    // std::cout << "---------------" << std::endl;
  }

  for(int i=0; i<10; i++){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    // transform.setOrigin(tf::Vector3(rhand_cam_x.at(9), rhand_cam_y.at(9), rhand_depth_vec_cam_coords.at(9)));
    transform.setOrigin(tf::Vector3(rhand_depth_vec_cam_coords.at(9), -rhand_cam_y.at(9), -rhand_cam_x.at(9)));
    tf::Quaternion quat(0, 0, 0, 1);
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_frame", "rhand9"));
    ros::Duration(0.001).sleep();
  }

  geometry_msgs::TransformStamped transform_stamped;
  try{
    transform_stamped = tf_buffer->lookupTransform("object", "rhand9", ros::Time(0));
  } catch(tf2::TransformException &ex) {
    ROS_WARN("tf2 error: %s", ex.what());
  }

  double obj_to_rhand9_x = transform_stamped.transform.translation.x;
  double obj_to_rhand9_y = transform_stamped.transform.translation.y;
  double obj_to_rhand9_z = transform_stamped.transform.translation.z;
  std::cout << rhand_cam_x.at(9) << ", " << rhand_cam_y.at(9) << ", " << rhand_depth_vec_cam_coords.at(9) << std::endl;
  // std::cout << obj_to_rhand9_x << ", " << obj_to_rhand9_y << ", " << obj_to_rhand9_z << std::endl;

}

std::vector<std::string> split(std::string& input, char delimiter){
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void readPose(int frame){
  std::stringstream file_name;
  std::ostringstream num;
  std::string num_;
  num << std::setfill('0') << std::setw(4) << frame;
  num_ = num.str();
  file_name << "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/csv/" << num_ << ".csv";
  
  std::vector<double> wrist;
  std::vector<double> lhand;
  std::vector<double> rhand;

  std::ifstream ifs(file_name.str());
  std::string line;

  for(int j=0; j<44; j++){
    getline(ifs, line);
    std::vector<std::string> strvec = split(line, ',');
    std::vector<double > vec((int)strvec.size());
    // std::cout << "---------------------------------" << std::endl << std::endl;

    for (int i=0; i<strvec.size();i++){
      try{
	vec[i] = std::stod(strvec[i]);
	// std::cout << vec[i] << ", " << i <<  std::endl;
	if(j == 0 || j == 1){
	  wrist.push_back(vec[i]);
	} else if(j > 1 && j <24) {
	  lhand.push_back(vec[i]);
	} else {
	  rhand.push_back(vec[i]);
	}
      } catch(std::invalid_argument e) {
	std::cout << "error! check csv file" << std::endl;
      }
    }
  }
  cv::Mat depth_img = readDepthImg(frame);
  get3dPosition(depth_img, wrist, lhand, rhand, frame);
}

cv::Mat readDepthImg(int frame) {
  cv::Mat depth_img;
  std::stringstream file_name;
  file_name << "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/openpose_video_2_0124/depth_img/depth_" << frame << ".xml";

  cv::FileStorage cvfs(file_name.str(), CV_STORAGE_READ);
  cv::FileNode node(cvfs.fs, NULL);
  cv::read(node["mat"], depth_img);
  cvfs.release();

  return depth_img;
}

void get3dPosition(cv::Mat &depth, std::vector<double> &wrist, std::vector<double> &lhand, std::vector<double> &rhand, int frame){
  cv::imshow("depth",depth);
  cv::waitKey(380);
  for(int i=0; i<(int)wrist.size();i++){
    // std::cout << wrist.at(i) << std::endl;
  }
  for(int i=0; i<(int)lhand.size();i++){
    // std::cout << lhand.at(i) << std::endl;
  }
  for(int i=0; i<(int)rhand.size();i++){
    // std::cout << rhand.at(i) << std::endl;
  }

  // printf("------------------------\n");
  std::vector<cv::Point> rhand_vec;
  std::vector<cv::Point> lhand_vec;
  std::vector<cv::Point> wrist_vec;

  for(int i=0; i<20; i++){ // 20 is magic number need debug
    int index = i*3;
    cv::Point rhand_2d_pos(rhand.at(index), rhand.at(index+1));
    rhand_vec.push_back(rhand_2d_pos);
  }

  for(int i=0; i<20; i++){ // 20 is magic number need debug
    int index = i*3;
    cv::Point lhand_2d_pos(lhand.at(index), lhand.at(index+1));
    lhand_vec.push_back(lhand_2d_pos);
  }

  for(int i=0; i<2; i++){ // 2 is magic number need debug
    int index = i*3;
    cv::Point wrist_2d_pos(wrist.at(index), wrist.at(index+1));
    wrist_vec.push_back(wrist_2d_pos);
  }

  // printf("**********************\n");
  cv::Mat display = depth.clone();
  // printf("----------------------\nrhand\n");
  std::vector<float> rhand_depth_vec;

  for(int i=0; i<20; i++){
    if(depth.at<cv::Vec3f>(rhand_vec.at(i).y, rhand_vec.at(i).x)[2]){
      rhand_depth_vec.push_back(depth.at<cv::Vec3f>(rhand_vec.at(i).y, rhand_vec.at(i).x)[2]);
    }
    // std::cout << rhand_vec.at(i).x << ", " << rhand_vec.at(i).y << ", " << rhand_depth_vec.at(i) <<std::endl;
    cv::circle(display, rhand_vec.at(i), 5, cv::Scalar(255,0,0), -1);
  }

  std::vector<cv::Point> rhand_vec_cam_coords;
  std::vector<float> rhand_depth_vec_cam_coords;
  std::copy(rhand_vec.begin(), rhand_vec.end(), std::back_inserter(rhand_vec_cam_coords));
  std::copy(rhand_depth_vec.begin(), rhand_depth_vec.end(), std::back_inserter(rhand_depth_vec_cam_coords));

  cameraPixelTrans(rhand_vec_cam_coords, rhand_depth_vec_cam_coords);

  // printf("----------------------\nlhand\n");
  std::vector<float> lhand_depth_vec;
  for(int i=0; i<20; i++){
    if(depth.at<cv::Vec3f>(lhand_vec.at(i).y, lhand_vec.at(i).x)[2]){
      lhand_depth_vec.push_back(depth.at<cv::Vec3f>(lhand_vec.at(i).y, lhand_vec.at(i).x)[2]);
    }
    // std::cout << lhand_vec.at(i).x << ", " << lhand_vec.at(i).y << ", " << lhand_depth_vec.at(i) <<std::endl;
    cv::circle(display, lhand_vec.at(i), 5, cv::Scalar(0,0,255), -1);
  }
  // printf("----------------------\nwrist\n");
  std::vector<float> wrist_depth_vec;
  for(int i=0; i<2; i++){
    if(depth.at<cv::Vec3f>(wrist_vec.at(i).y, wrist_vec.at(i).x)[2]){
      wrist_depth_vec.push_back(depth.at<cv::Vec3f>(wrist_vec.at(i).y, wrist_vec.at(i).x)[2]);
    }
    cv::circle(display, wrist_vec.at(i), 5, cv::Scalar(0,255, 0), -1);
    // std::cout << wrist_vec.at(i).x << ", " << wrist_vec.at(i).y << ", " << wrist_depth_vec.at(i) <<std::endl;
  }
  cv::imshow("display", display);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "keypoints_csv");

  tf2_ros::Buffer tf_buffer_ptr;
  tf_buffer = &tf_buffer_ptr;
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  // for(int frame=50; frame<1068; frame++){
  // for(int frame=430; frame<540; frame++){
  for(int frame=375; frame<405; frame++){
    int c = cv::waitKey(20);
    if(c == 27){ //esc
      return -1;
    } else if(c == 0x73) { //"s"
      cv::waitKey(0);
    }
    printf("%d, ", frame);
    readPose(frame);
  }
  cv::waitKey(0);
  return 0;
}
