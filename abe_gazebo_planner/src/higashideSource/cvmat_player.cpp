#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define START 70
#define END 400

std::string path = "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/";
std::string color_name = path + "proceeding0228_1/assemble_video0228/color_img/color_";
std::string video_name = path + "proceeding0228_1/assemble_video0228/depth_img/depth_";
std::ifstream ifs("/home/ais/ais-project/catkin_ws/src/devel/abe/higa_sandbox/scripts/motion_probability0228_1.csv");
std::string line;

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
  pose_file << video_name << frame << ".xml";
  cv::FileStorage cvfs(pose_file.str(), CV_STORAGE_READ);
  cv::FileNode node(cvfs.fs, NULL);
  cv::read(node["mat"], depth_img);
  cvfs.release();
  return depth_img;
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

void readProbabilityData(int index){
  getline(ifs, line);
  std::vector<std::string> strvec = split(line, ' ');

  std::vector<double > vec((int)strvec.size());
  std::cout << "---------------------------------" << std::endl << std::endl;;
  for (int i=0; i<strvec.size();i++){
    try{
      vec[i] = std::stod(strvec[i]);
      // std::cout << vec[i] << std::endl;
    } catch(std::invalid_argument e) {
      std::cout << "error! check csv file" << std::endl;
    }
  }

  int frame = vec[0]-7;

  cv::Mat depth_img = readDepthImg(frame);
  cv::Mat color_img = readColorImg(frame);

  double screw = vec[1]*100;
  double insert = vec[2]*100;
  double other = vec[3]*100;

  std::map<double, std::string> probability;
  probability[screw] = "screw";
  probability[insert] = "insert";
  probability[other] = "other";

  double max_probability = (std::max)({screw, insert, other});
  max_probability = max_probability;
  std::cout << max_probability << std::endl;
  std::string motion = probability[max_probability];
  std::cout << motion << std::endl;

  std::string screw_text = "screw: " + std::to_string(screw)+"%";
  cv::putText(color_img, screw_text, cv::Point2i(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(0, 0, 0), 2);
  std::string insert_text = "insert: " + std::to_string(insert)+"%";
  cv::putText(color_img, insert_text, cv::Point2i(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(0, 0, 0), 2);
  std::string other_text = "other: " + std::to_string(other)+"%";
  cv::putText(color_img, other_text, cv::Point2i(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(0, 0, 0), 2);
  
  std::string frame_text =  "frame: " + std::to_string(frame);
  cv::putText(color_img, frame_text, cv::Point2i(50, color_img.rows/2+100), cv::FONT_HERSHEY_SIMPLEX, 1.6, cv::Scalar(0, 0, 0), 4);
  std::string motion_text =  "motion: " + motion;
  cv::putText(color_img, motion_text, cv::Point2i(50, color_img.rows/2), cv::FONT_HERSHEY_SIMPLEX, 1.6, cv::Scalar(0, 0, 0), 4);

  cv::imshow("color_img", color_img);
  cv::waitKey(20);
}

int main(int argc, char **argv){
  for(int index=0; index<END; index++){
    int c = cv::waitKey(20);
    if(c == 27){ //esc
      return -1;
    } else if(c == 0x73) { //"s"
      cv::waitKey(0);
    }
    readProbabilityData(index);
  }
  cv::waitKey(0);
  return 0;
}
