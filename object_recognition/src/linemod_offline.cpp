#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <math.h>
// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#if CV_MAJOR_VERSION < 3

// global variable
cv::Ptr<cv::linemod::Detector> detector;
std::string filename;
std::string template_path = "/home/ais/ais-project/catkin_ws/src/devel/abe/abe_linemod_pkg/linemod_template/";
int num_classes = 0;

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);
// void split3dPose(cv::linemod::Match m, double& roll, double& pitch, double& yaw);
void split3dPose(cv::linemod::Match m, double& quat_x, double& quat_y, double& quat_z, double& quat_w);

class Mouse {
public:
  static void start(const std::string& a_img_name) {
    cvSetMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
  }
  static int event(void) {
    int l_event = m_event;
    m_event = -1;
    return l_event;
  }
  static int x(void) {
    return m_x;
  }
  static int y(void) {
    return m_y;
  }
private:
  static void cv_on_mouse(int a_event, int a_x, int a_y, int, void *) {
    m_event = a_event;
    m_x = a_x;
    m_y = a_y;
  }

  static int m_event;
  static int m_x;
  static int m_y;
};
int Mouse::m_event;
int Mouse::m_x;
int Mouse::m_y;

class Timer {
public:
  Timer() : start_(0), time_(0) {}

  void start() {
    start_ = cv::getTickCount();
  }

  void stop() {
    CV_Assert(start_ != 0);
    int64 end = cv::getTickCount();
    time_ += end - start_;
    start_ = 0;
  }

  double time() {
    double ret = time_ / cv::getTickFrequency();
    time_ = 0;
    return ret;
  }
private:
  int64 start_, time_;
};

static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename) {
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());
  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);
  return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename) {
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);
  std::vector<std::string> ids = detector->classIds();
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i) {
    fs << "{";
    detector->writeClass(ids[i], fs);
    fs << "}"; // current class
  }
  fs << "]"; // classes
}

// void split3dPose(cv::linemod::Match m, double& roll, double& pitch, double& yaw) {
void split3dPose(cv::linemod::Match m, double& quat_x, double& quat_y, double& quat_z, double& quat_w) {
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

  std::cerr << class_id << std::endl;
  for(std::string i : list){
    std::cout << i << std::endl;
  }
  quat_x = std::stod(list.at(1));
  quat_y = std::stod(list.at(2));
  quat_z = std::stod(list.at(3));
  quat_w = std::stod(list.at(4));
}

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T) {
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 255, 0),
                                        CV_RGB(0, 0, 255),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m) {
    cv::Scalar color = COLORS[m];
    cv::Point obj_center(offset.x+templates[m].width/2, offset.y+templates[m].height/2);
    cv::Point obj_center_with_offset(offset.x+templates[m].width/2+40, offset.y+templates[m].height/2+40);
    cv::circle(dst, obj_center, 3, cv::Scalar(0,0,255), -1);

    for (int i = 0; i < (int)templates[m].features.size(); ++i) {
      cv::rectangle(dst, cv::Point(offset.x, offset.y), cv::Point(offset.x+templates[m].width, offset.y+templates[m].height), CV_RGB(180,180,180), 1);
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T, color);
    }
  }
}

class rosLinemod {
private:
  int matching_threshold = 90;
  geometry_msgs::PoseStamped obj_pose_;
  geometry_msgs::Quaternion quat_pose;
  cv::Mat depth;

  ros::NodeHandle nh_;
  ros::Publisher obj_pose_pub_;
  void videoLinemod();

public:
  rosLinemod();
  ~rosLinemod();
};

rosLinemod::rosLinemod() : nh_() {
  obj_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("linemod/obj_pose", 1);

  obj_pose_.pose.position.x = 0.0;
  obj_pose_.pose.position.y = 0.0; 
  obj_pose_.pose.position.z = 0.0;
  obj_pose_.pose.orientation.x = 0.0;
  obj_pose_.pose.orientation.y = 0.0; 
  obj_pose_.pose.orientation.z = 0.0;
  obj_pose_.pose.orientation.w = 0.0;

  videoLinemod();
}

rosLinemod::~rosLinemod(){}

void rosLinemod::videoLinemod(){
  bool show_match_result = true;
  bool show_timings = false;
  bool learn_online = false;

  cv::Size roi_size(200, 200);
  int learning_lower_bound = 90;
  int learning_upper_bound = 95;

  // Timers
  Timer extract_timer;
  Timer match_timer;
  int num_modalities = (int)detector->getModalities().size();

  cv::VideoCapture capture("/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/openpose_video_2_0124/human_pose_RGB.avi");
  if (!capture.isOpened()){
    printf("Could not open video\n");
    return;
  }
  cv::Mat color;
  while(ros::ok){
    capture.grab();
    capture.retrieve(color, 5);

    cv::imshow("color_test", color);
    cv::waitKey(20);

    double focal_length = 500;
    std::vector<cv::Mat> sources;
    sources.push_back(color);
    //  sources.push_back(depth);
    cv::Mat display = color.clone();
  
    if (!learn_online) {
      cv::Point mouse(Mouse::x(), Mouse::y());
      int event = Mouse::event();

      // Compute ROI centered on current mouse location
      cv::Point roi_offset(roi_size.width / 2, roi_size.height / 2);
      cv::Point pt1 = mouse - roi_offset; // top left
      cv::Point pt2 = mouse + roi_offset; // bottom right
      std::vector<CvPoint> chain(4);
      chain[0] = pt1;
      chain[1] = cv::Point(pt2.x, pt1.y);
      chain[2] = pt2;
      chain[3] = cv::Point(pt1.x, pt2.y);

      // Draw ROI for display
      cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);
      cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 1);
    }

    // Perform matching
    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;
    match_timer.start();
    detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);
    match_timer.stop();

    int classes_visited = 0;
    std::set<std::string> visited;

    for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i) {
      // cv::linemod::Match m = matches[i];
      cv::linemod::Match m = matches[0];
      const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);

      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      // pose obj3dpose;
      double quat_x = 0;
      double quat_y = 0;
      double quat_z = 0;
      double quat_w = 0;

      if (visited.insert(m.class_id).second) {
	++classes_visited;

	if (show_match_result) {
	  printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
		 m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
	  // split3dPose(m, roll, pitch, yaw);
	  split3dPose(m, quat_x, quat_y, quat_z, quat_w);
	  std::cout << quat_x << ", " << quat_y << ", " << quat_z << ", " << quat_w << std::endl;
	  // std::cout << roll*180/M_PI << ", " << pitch*180/M_PI << ", " << yaw*180/M_PI << std::endl;
	}
	drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));

	int similarity = (int)m.similarity; 
	std::string result_text = std::string(std::to_string(similarity))+"%";
	cv::putText(display, result_text, cv::Point2i(m.x, m.y), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(100,200, 0), 1);

	cv::Point obj_center(matches[0].x + templates[0].width/2, matches[0].y + templates[0].height/2);
	cv::Point obj_center_with_offset(matches[0].x + templates[0].width/2+40, matches[0].y + templates[0].height/2+40);
	// cv::Point img_position(matches[0].x, matches[0].y);

	float obj_z = 0.6;
	obj_pose_.pose.position.x = obj_center.x;
	obj_pose_.pose.position.y = obj_center.y;
	obj_pose_.pose.position.z = obj_z;

	// tf::Quaternion quat = tf::createQuaternionFromRPY(roll,pitch,yaw);
	tf::Quaternion quat = tf::createQuaternionFromRPY(-yaw, -pitch, roll);
	geometry_msgs::Quaternion quat_msg;
	quaternionTFToMsg(quat, quat_msg); // convert tf_quat to geometry_msgs_quat

	// obj_pose_.pose.orientation = quat_msg;
	obj_pose_.pose.orientation.x = quat_x;
	obj_pose_.pose.orientation.y = quat_y;
	obj_pose_.pose.orientation.z = quat_z;
	obj_pose_.pose.orientation.w = quat_w;

	obj_pose_pub_.publish(obj_pose_);      
      }
    }

    if (show_match_result && matches.empty())
      printf("No matches found...\n");
    if (show_timings) {
      printf("Training: %.2fs\n", extract_timer.time());
      printf("Matching: %.2fs\n", match_timer.time());
    }
    if (show_match_result || show_timings)
      printf("------------------------------------------------------------\n");

    cv::imshow("color", display);
    // cv::imshow("normals", quantized_images[1]);
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
    case 'w':
      // write model to disk
      writeLinemod(detector, filename);
      printf("Wrote detector and templates to %s\n", filename.c_str());
      break;
    default:
      ;
    }
  }
}

#endif
int main(int argc, char** argv) {
  ros::init(argc, argv, "linemod");
  cv::namedWindow("color", CV_WINDOW_NORMAL);
  cv::namedWindow("normals");
  Mouse::start("color");

  if (argc == 1) {
    filename = "linemod_templates.yml";
    // use color edge and depth normal vector
    //detector = cv::linemod::getDefaultLINEMOD(); 
    // use color edge 
    detector = cv::linemod::getDefaultLINE(); 
  } else {
    detector = readLinemod(argv[1]);

    std::vector<std::string> ids = detector->classIds();
    num_classes = detector->numClasses();
    printf("Loaded %s with %d classes and %d templates\n", argv[1], num_classes, detector->numTemplates());
    if (!ids.empty()) {
      printf("Class ids:\n");
      std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    }
  }
  rosLinemod lm;
  return 0;
}


