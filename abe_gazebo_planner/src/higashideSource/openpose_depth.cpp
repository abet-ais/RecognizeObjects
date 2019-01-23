#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <time.h>
#include <stdexcept>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <numeric>
#include <iterator> 
#include <set>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// for xtion
// #define CX 319.5
// #define CY 239.5
// #define FX 570.3422241210938
// #define FY 570.3422241210938

// for realsense d435
#define CX 636.2023315429688
#define CY 371.5960388183594
#define FX 918.495849609375
#define FY 918.3920288085938

#define BACKGROUND_FILTER 1
#define FRONT_FILTER 0.1
#define BB_AREA 10
#define START 70
#define END 415

tf::TransformListener* listener_ptr;
tf2_ros::Buffer* tf_buffer;
FILE *fp;

std::string path = "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/";
std::string color_name = path + "proceeding0228_1/assemble_video0228/color_img/color_";
std::string video_name = path + "proceeding0228_1/assemble_video0228/depth_img/depth_";
std::string keypoints_name = path + "proceeding0228_1/keypoints0228/";
std::string output_file = path + "proceeding0228_1/output/";

void get3dPosition(cv::Mat &depth, cv::Mat &color, cv::Mat &pose, cv::Mat &lhand, cv::Mat &rhand, int frame);
void cameraPixelTrans(std::vector<cv::Point>& rhand_vec_cam_coords, std::vector<float>& rhand_depth_vec_cam_coords, std::vector<float>& rhand_score_vec, int frame);
cv::Mat readDepthImg(int frame);
cv::Mat readColorImg(int frame);
void writeCsv(float x, float y, float z, float score, int frame, int index);
std::vector<float> serchDepth(int bb_area, const cv::Mat& depth, const std::vector<cv::Point>& rhand_vec, int i);

static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename) {
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

class Linemod {
private:
  cv::Mat color;
  cv::Mat depth;
  int matching_threshold;
  cv::Ptr<cv::linemod::Detector> detector;
  geometry_msgs::TransformStamped obj_to_parts;

  std::string template_path = "/home/ais/ais-project/catkin_ws/src/devel/abe/abe_linemod_pkg/linemod_template0216/";

  int num_classes;
  float match;
  cv::Point obj_center;
  int sigmoid_threshold[3];
  // void linemodDetect(cv::Mat& color, const cv::Mat& depth, geometry_msgs::TransformStamped& transform_stamped, float& match, cv::Point& obj_center);
  void linemodDetect(cv::Mat& color, const cv::Mat& depth, float& match, cv::Point& obj_center, int sigmoid);
  void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);
  void split3dPose(cv::linemod::Match m, double& quat_x, double& quat_y, double& quat_z, double& quat_w);
  void writeLinemodCsv(cv::Point& obj_center, float match, int frame, int i);
  std::vector<std::string> linemod_file;

public:
  Linemod();
  ~Linemod();
  void setLinemod(cv::Mat color_img, cv::Mat depth_img, int frame);
};

Linemod::Linemod(){
  matching_threshold = 83;
  num_classes = 0;
  match = 0;
  obj_center = cv::Point(0, 0);
  linemod_file.push_back(template_path+"shaft.yml");
  linemod_file.push_back(template_path+"base.yml");
  linemod_file.push_back(template_path+"lgear2.yml");
  sigmoid_threshold[0] = 88;
  sigmoid_threshold[1] = 80;
  sigmoid_threshold[2] = 80;
}

Linemod::~Linemod(){
  std::cout << "**************************************************" << std::endl;
}

void Linemod::setLinemod(cv::Mat color_img, cv::Mat depth_img, int frame){
  cv::namedWindow("color", CV_WINDOW_NORMAL);
  color = color_img.clone();
  depth = depth_img.clone();
  for(int i=0; i<linemod_file.size(); i++){
    printf("------------------------------------\n");  
    detector = readLinemod(linemod_file.at(i));
    std::vector<std::string> ids = detector->classIds();
    num_classes = detector->numClasses();
    match = 0;
    obj_center = cv::Point(0, 0);
    int sigmoid = sigmoid_threshold[i];
    // linemodDetect(color, depth, obj_to_parts, match, obj_center);
    linemodDetect(color, depth, match, obj_center, sigmoid);
    // writeLinemodCsv(obj_center , match, frame, i);
  }
}

void Linemod::writeLinemodCsv(cv::Point& obj_center, float match, int frame, int i){
  float u = obj_center.x;
  float v = obj_center.y;
  // float x = obj_to_parts.transform.translation.x;
  // float y = obj_to_parts.transform.translation.y;
  // float z = obj_to_parts.transform.translation.z;
  float x = 0;
  float y = 0;
  float z = 0;

  std::string delimiter = ", ";
  std::string coords = 
    (std::to_string(frame)) + delimiter + 
    (std::to_string(u)) + delimiter + 
    (std::to_string(v)) + delimiter + 
    (std::to_string(x)) + delimiter + 
    (std::to_string(y)) + delimiter + 
    (std::to_string(z)) + delimiter +
    (std::to_string(match));
  
  std::cout << coords << std::endl;

  char filename[500];
  std::string output = output_file+"linemod_info"+std::to_string(i)+".csv";
  sprintf(filename, output.c_str());
  fp = fopen(filename, "a");
  if(fp == NULL){
    fprintf(stderr, "error\n");
    return;
  }
  fprintf(fp, coords.c_str());
  fprintf(fp, "\n");
  fclose(fp);
}

void Linemod::split3dPose(cv::linemod::Match m, double& quat_x, double& quat_y, double& quat_z, double& quat_w) {
  std::string class_id = m.class_id.c_str();

  auto string = class_id;    // split string
  auto separator = std::string(",");         // delimiter
  auto separator_length = separator.length(); // split string length
  auto list = std::vector<std::string>(); // vector store each splited string

  if (separator_length == 0) {
    list.push_back(string);
  } else {
    auto offset = std::string::size_type(0);
    while (1) {
      auto pos = string.find(separator, offset);
      if (pos == std::string::npos) {
	list.push_back(string.substr(offset));
	break;
      }
      list.push_back(string.substr(offset, pos - offset));
      offset = pos + separator_length;
    }
  }

  for(std::string i : list){
    // std::cout << i << std::endl;
  }
  quat_x = std::stod(list.at(1));
  quat_y = std::stod(list.at(2));
  quat_z = std::stod(list.at(3));
  quat_w = std::stod(list.at(4));
}

void Linemod::drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T) {
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 255, 0),
                                        CV_RGB(0, 0, 255),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m) {
    // NOTE: Original demo recalculated max response for each feature in the TxT
    // box around it and chose the display color based on that response. Here
    // the display color just depends on the modality.
    cv::Scalar color = COLORS[m];
    cv::Point obj_center(offset.x+templates[m].width/2, offset.y+templates[m].height/2);
    cv::Point obj_center_with_offset(offset.x+templates[m].width/2+40, offset.y+templates[m].height/2+40);
    cv::circle(dst, obj_center, 3, cv::Scalar(0,0,255), -1);

    for (int i = 0; i < (int)templates[m].features.size(); ++i) {
      cv::rectangle(dst, cv::Point(offset.x, offset.y), cv::Point(offset.x+templates[m].width, offset.y+templates[m].height), CV_RGB(180,180,180), 1);
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T/2, color, -1);
    }
  }
}
 
void Linemod::linemodDetect(cv::Mat& color, const cv::Mat& depth, float& match, cv::Point& obj_center, int sigmoid){
  bool show_match_result = true;
  int num_modalities = (int)detector->getModalities().size();

  // cut background
  cv::Mat linemod_display = color.clone();
  cv::rectangle(linemod_display, cv::Point(0,0), cv::Point(linemod_display.cols,6*linemod_display.rows/10), cv::Scalar(0,0,0), -1, CV_AA);
  // cv::rectangle(linemod_display, cv::Point(0,450), cv::Point(linemod_display.cols,linemod_display.rows), cv::Scalar(0,0,0), -1, CV_AA);
  cv::rectangle(linemod_display, cv::Point(0,0), cv::Point(1*linemod_display.cols/4,linemod_display.rows), cv::Scalar(0,0,0), -1, CV_AA);
  cv::rectangle(linemod_display, cv::Point(5*linemod_display.cols/6,0), cv::Point(linemod_display.cols,linemod_display.rows), cv::Scalar(0,0,0), -1, CV_AA);

  std::vector<cv::Mat> sources;
  sources.push_back(linemod_display);

  // Perform matching
  std::vector<cv::linemod::Match> matches;
  std::vector<std::string> class_ids;
  std::vector<cv::Mat> quantized_images;
  detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);

  int classes_visited = 0;
  std::set<std::string> visited;

  for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i) {
    cv::linemod::Match m = matches[0];
    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);

    double quat_x = 0;
    double quat_y = 0;
    double quat_z = 0;
    double quat_w = 0;

    if (visited.insert(m.class_id).second) {
      ++classes_visited;

      drawResponse(templates, num_modalities, linemod_display, cv::Point(m.x, m.y), detector->getT(0));

      int similarity = (int)m.similarity; 
      match = m.similarity;      
      // float sig = exp((-1.1)*(m.similarity - sigmoid));
      // match = 1/(1+sig);
      std::cout << match << ", " << m.similarity << std::endl;

      std::string result_text = std::string(std::to_string(similarity))+"%";
      cv::putText(linemod_display, result_text, cv::Point2i(m.x, m.y), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(100,200, 0), 1);

      obj_center = cv::Point(matches[0].x + templates[0].width/2, matches[0].y + templates[0].height/2);

      /*
      float cam_x;
      float cam_y;
      float obj_z = 0.65;
      cam_x = (obj_z)*(CX-obj_center.x)/FX;
      cam_y = (obj_z)*(CY-obj_center.y)/FY;

      for(int j=0; j<10; j++){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(obj_z, cam_x, cam_y));
	tf::Quaternion quat(quat_x, quat_y, quat_z, quat_w);
	transform.setRotation(quat);
	std::string parent = "camera_link";
	std::string child_frame = "parts";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, child_frame));
	ros::Duration(0.001).sleep();
      }
      std::string parent = "object";
      std::string child_frame = "parts";
      try{
	transform_stamped = tf_buffer->lookupTransform(parent, child_frame, ros::Time(0));
      } catch(tf2::TransformException &ex) {
	ROS_WARN("tf2 error: %s", ex.what());
      }
      */
    }
  }

  if (show_match_result && matches.empty()){
    printf("No matches found...\n");
  } else {

  }
  cv::imshow("linemod_color", linemod_display);
  cv::waitKey(10);
  cv::FileStorage fs;
  char key = (char)cvWaitKey(10);

  switch (key) {
  case '[':
    // decrement threshold
    matching_threshold = std::max(matching_threshold - 1, -100);
    printf("New threshold: %d\n", matching_threshold);
    break;
  case ']':
    // increment threshold
    matching_threshold = std::min(matching_threshold + 1, +100);
    printf("New threshold: %d\n", matching_threshold);
    break;
  default:
    ;
  }
}

void get3dPosition(cv::Mat &depth, cv::Mat &color, cv::Mat &pose, cv::Mat &lhand, cv::Mat &rhand, int frame){
  Linemod linemod;

  std::vector<cv::Point> rhand_vec;
  std::vector<cv::Point> lhand_vec;
  std::vector<cv::Point> pose_vec;
  std::vector<float> rhand_score_vec;

  for(int i=0; i<21; i++){
    cv::Point rhand_2d_pos(rhand.at<float>(0, i, 0), rhand.at<float>(0, i, 1));
    float tmp_score = rhand.at<float>(0, i, 2);
    rhand_vec.push_back(rhand_2d_pos);
    rhand_score_vec.push_back(tmp_score);
  }
  for(int i=0; i<21; i++){
    cv::Point lhand_2d_pos(lhand.at<float>(0, i, 0), lhand.at<float>(0, i, 1));
    lhand_vec.push_back(lhand_2d_pos);
  }
  for(int i=0; i<18; i++){
    cv::Point pose_2d_pos(pose.at<float>(0, i, 0), pose.at<float>(0, i, 1));
    pose_vec.push_back(pose_2d_pos);
  }

  cv::Mat display = depth.clone();

  std::vector<float> rhand_depth_vec;
  int bb_area = BB_AREA;
  // int bb_area = 10;
  for(int i=0; i<21; i++){
    std::vector<float> depth_buf = serchDepth(bb_area, depth, rhand_vec, i);

    float depth_result = 0;
    float tmp_result = 0;

    if(depth_buf.size() != 0){
      for(int i=0; i<depth_buf.size(); i++){
	tmp_result += depth_buf.at(i);
      }
      depth_result = tmp_result/(float)depth_buf.size();
    }
    rhand_depth_vec.push_back(depth_result);

    cv::circle(color, rhand_vec.at(i), 2, cv::Scalar(0,0,255), -1);
    cv::putText(color, std::to_string(i), cv::Point(rhand_vec.at(i).x, rhand_vec.at(i).y+10), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0,0,255), 1);
  }

  // std::cout << frame << " depth_at(9): " << rhand_depth_vec.at(9) << std::endl;

  cv::putText(color, "frame: "+std::to_string(frame), cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,0), 1);
  
  std::vector<cv::Point> rhand_vec_cam_coords;
  std::vector<float> rhand_depth_vec_cam_coords;
  std::copy(rhand_vec.begin(), rhand_vec.end(), std::back_inserter(rhand_vec_cam_coords));
  std::copy(rhand_depth_vec.begin(), rhand_depth_vec.end(), std::back_inserter(rhand_depth_vec_cam_coords));
  cameraPixelTrans(rhand_vec_cam_coords, rhand_depth_vec_cam_coords, rhand_score_vec, frame);

  std::vector<float> lhand_depth_vec;
  for(int i=0; i<21; i++){
    if(depth.at<cv::Vec3f>(lhand_vec.at(i).y, lhand_vec.at(i).x)[2]){
      lhand_depth_vec.push_back(depth.at<cv::Vec3f>(lhand_vec.at(i).y, lhand_vec.at(i).x)[2]);
    }
  }

  std::vector<cv::Point> lhand_vec_cam_coords;
  std::vector<float> lhand_depth_vec_cam_coords;
  std::copy(lhand_vec.begin(), lhand_vec.end(), std::back_inserter(lhand_vec_cam_coords));
  std::copy(lhand_depth_vec.begin(), lhand_depth_vec.end(), std::back_inserter(lhand_depth_vec_cam_coords));

  std::vector<float> pose_depth_vec;
  for(int i=0; i<18; i++){
    if(depth.at<cv::Vec3f>(pose_vec.at(i).y, pose_vec.at(i).x)[2]){
      pose_depth_vec.push_back(depth.at<cv::Vec3f>(pose_vec.at(i).y, pose_vec.at(i).x)[2]);
    }
  }
  cv::imshow("display", display);
  cv::imshow("color", color);

  std::cout << "start linemod" << std::endl;
  linemod.setLinemod(color, depth, frame);
  std::cout << "end linemod" << std::endl;
}

std::vector<float> serchDepth(int bb_area, const cv::Mat& depth, const std::vector<cv::Point>& rhand_vec, int i){
  std::vector<float> tmp_depth_buf;
  cv::Point top_left(rhand_vec.at(i).x-(bb_area/2), rhand_vec.at(i).y-(bb_area/2));
  for(int i=0; i<bb_area; i++){
    for(int j=0; j<bb_area; j++){
      float tmp_depth = (depth.at<cv::Vec3f>(top_left.y+j, top_left.x+i)[2]) / 1000;
      if(tmp_depth < BACKGROUND_FILTER && tmp_depth > FRONT_FILTER){
	tmp_depth_buf.push_back(tmp_depth);
      }
    }
  }
  if(tmp_depth_buf.size()!=0 || bb_area > 200){
    return tmp_depth_buf;
  } else {
    return serchDepth(bb_area+5, depth, rhand_vec, i);
  }
}

void cameraPixelTrans(std::vector<cv::Point>& rhand_vec_cam_coords, std::vector<float>& rhand_depth_vec_cam_coords, std::vector<float>& rhand_score_vec, int frame){

  std::vector<float> rhand_cam_x;
  std::vector<float> rhand_cam_y;
  
  for(int i=0; i<21; i++){
    float tmp_rhand_cam_x;
    float tmp_rhand_cam_y;
    float tmp_rhand_depth_cam;
    tmp_rhand_cam_x = (rhand_depth_vec_cam_coords.at(i)*(CX-rhand_vec_cam_coords.at(i).x))/FX;
    tmp_rhand_cam_y = (rhand_depth_vec_cam_coords.at(i)*(CY-rhand_vec_cam_coords.at(i).y))/FY;
    rhand_cam_x.push_back(tmp_rhand_cam_x);
    rhand_cam_y.push_back(tmp_rhand_cam_y);
  }

  std::vector<float> obj_to_rhand_x;
  std::vector<float> obj_to_rhand_y;
  std::vector<float> obj_to_rhand_z;

  for(int i=0; i<rhand_cam_x.size(); i++){
    for(int j=0; j<10; j++){
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(rhand_depth_vec_cam_coords.at(i), rhand_cam_x.at(i), rhand_cam_y.at(i)));
      tf::Quaternion quat(0, 0, 0, 1);
      transform.setRotation(quat);
      std::string parent = "camera_link";
      std::string child_frame = "rhand"+std::to_string(i);
      // std::string child_frame = std::to_string(i);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, child_frame));
      ros::Duration(0.001).sleep();
    }
    geometry_msgs::TransformStamped transform_stamped;
    std::string parent = "object";
    std::string child_frame = "rhand"+std::to_string(i);
    try{
      transform_stamped = tf_buffer->lookupTransform(parent, child_frame, ros::Time(0));
      obj_to_rhand_x.push_back(transform_stamped.transform.translation.x);
      obj_to_rhand_y.push_back(transform_stamped.transform.translation.y);
      obj_to_rhand_z.push_back(transform_stamped.transform.translation.z);
    } catch(tf2::TransformException &ex) {
      ROS_WARN("tf2 error: %s", ex.what());
    }
  }

  for(int i=0; i<obj_to_rhand_x.size(); i++){
    // std::cout << i << ", " << obj_to_rhand_x.at(i) << std::endl;
    // writeCsv(obj_to_rhand_x.at(i), obj_to_rhand_y.at(i), obj_to_rhand_z.at(i), rhand_score_vec.at(i), frame, i);    
  }
}

void writeCsv(float x, float y, float z, float score, int frame, int index){
  std::string delimiter = ", ";
  std::string coords = 
    (std::to_string(frame)) + delimiter + 
    (std::to_string(x)) + delimiter + 
    (std::to_string(y)) + delimiter + 
    (std::to_string(z)) + delimiter +
    (std::to_string(score));


  char filename[500];
  std::string output = output_file+"rhand"+std::to_string(index)+".csv";
  sprintf(filename, output.c_str());
  fp = fopen(filename, "a");
  if(fp == NULL){
    fprintf(stderr, "error\n");
    return;
  }
  fprintf(fp, coords.c_str());
  fprintf(fp, "\n");
  fclose(fp);
}

void readPose(int frame){
  std::stringstream pose_file;
  std::ostringstream num;
  std::string num_;
  num << std::setfill('0') << std::setw(4) << frame;
  num_ = num.str();
  // std::cout << num_ << std::endl;
  std::string path = video_name;
  pose_file << keypoints_name << "human_pose_RGB_00000000" << num_ << "_pose.yml";
  cv::FileStorage posefs(pose_file.str(), cv::FileStorage::READ);
  cv::FileNode pose_node(posefs.fs, NULL);
  cv::Mat pose;
  cv::read(pose_node["pose_0"], pose);
  posefs.release();

  std::stringstream rhand_file;
  rhand_file << keypoints_name << "human_pose_RGB_00000000" << num_ << "_hand_right.yml";
  cv::FileStorage rhandfs(rhand_file.str(), cv::FileStorage::READ);
  cv::FileNode rhand_node(rhandfs.fs, NULL);
  cv::Mat rhand;
  cv::read(rhand_node["hand_right_0"], rhand);
  rhandfs.release();

  std::stringstream lhand_file;
  lhand_file << keypoints_name << "human_pose_RGB_00000000" << num_ << "_hand_left.yml";
  cv::FileStorage lhandfs(lhand_file.str(), cv::FileStorage::READ);
  cv::FileNode lhand_node(lhandfs.fs, NULL);
  cv::Mat lhand;
  cv::read(lhand_node["hand_left_0"], lhand);
  lhandfs.release();

  cv::Mat depth_img = readDepthImg(frame);
  cv::Mat color_img = readColorImg(frame);

  if(rhand.cols!=0 && lhand.cols!=0 && pose.cols!=0)
    get3dPosition(depth_img, color_img, pose, lhand, rhand, frame);
}

cv::Mat readColorImg(int frame){
  cv::Mat color_img;
  std::stringstream pose_file;

  pose_file << color_name << frame << ".xml";
  cv::FileStorage cvfs(pose_file.str(), CV_STORAGE_READ);
  cv::FileNode node(cvfs.fs, NULL);
  cv::read(node["mat"], color_img);
  cvfs.release();
  return color_img;
}

cv::Mat readDepthImg(int frame) {
  cv::Mat depth_img;
  std::stringstream pose_file;
  // pose_file << keypoints_name << frame << ".xml";
  pose_file << video_name << frame << ".xml";
  cv::FileStorage cvfs(pose_file.str(), CV_STORAGE_READ);
  cv::FileNode node(cvfs.fs, NULL);
  cv::read(node["mat"], depth_img);
  cvfs.release();
  return depth_img;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "openpose_linemod");
  
  tf2_ros::Buffer tf_buffer_ptr;
  tf_buffer = &tf_buffer_ptr;
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  for(int frame=START; frame<END; frame++){
    int c = cv::waitKey(20);
    if(c == 27){ //esc
      return -1;
    } else if(c == 0x73) { //"s"
      cv::waitKey(0);
    }
    readPose(frame);
  }
  cv::waitKey(0);
  return 0;
}
