#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <iomanip>

void readPose(){
  cv::FileStorage cvfs("~/ais-project/catkin_ws/src/devel/abe/abe_sandbox/scripts/openpose_keypoints2.yml", cv::FileStorage::READ);
  cv::FileNode body = cvfs["body"];
  // cv::FileNodeIterator it = body.begin(), it_end = body.end();
  int idx = 0;
  std::vector<uchar> lbpval;

  // std::cerr << (int)it_end << std::endl;
  // iterate through a sequence using FileNodeIterator
  std::cerr << "!!!!!!!!!!!!!!1" << std::endl;
  // for(; it != it_end; ++it) {
  for(cv::FileNodeIterator it = body.begin(), itend = body.end(); it != itend; ++it){
    std::cerr << "aaaaaaaaaaaaa" << std::endl;

    std::cout << "body" << idx << ": ";
    std::cout <<  "lbp: (";
    // you can also easily read numerical arrays using FileNode >> std::vector operator.
    (*it)["lbp"] >> lbpval;
    for( int i = 0; i < (int)lbpval.size(); i++ )
      std::cout << " " << (int)lbpval[i];
    std::cout << ")" << std::endl;
  }
  cvfs.release();
}

cv::Mat readDepthImg(int frame) {
  cv::Mat depth_img;

  std::stringstream file_name;
  file_name << "~/ais-project/catkin_ws/src/devel/abe/openpose_data/openpose_test_0123_3/depth_img/depth_" << frame << ".xml";

  cv::FileStorage cvfs(file_name.str(), CV_STORAGE_READ);
  cv::FileNode node(cvfs.fs, NULL);

  cv::read(node["mat"], depth_img);
  cvfs.release();
  
  return depth_img;
}

/*
int save_depth_info(int frame, Mat joints) {
  std::stringstream file_name;
  file_name << "joints_info/joint_info_" << frame << ".xml";
  FileStorage fs(file_name.str(), FileStorage::WRITE);
  
  fs << "joint_info" << joints;
  fs.release();
}
*/

int main() {
  // for(int frame=40; frame<176; frame++){
  int frame = 570;
  cv::Mat depth_img = readDepthImg(frame);
  readPose();
  std::cout << frame << std::endl;
  // save_depth_info(frame, joints);
  // }
  return 0;
}
