#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

// #define SCALE_CHANGE 1158
#define SCALE_CHANGE 4381
// #define SCALE_CHANGE 2358
#define CANNY_VAL1 20
#define CANNY_VAL2 40
#define INITIAL_DIST 0.7
#define TF_OFFSET 5

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);

class Mouse{
public:
  static void start(const std::string& a_img_name){
    cvSetMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
  }
  static int event(void){
    int l_event = m_event;
    // m_event = -1;
    return l_event;
  }
  static int x(void){
    return m_x;
  }
  static int y(void) {
    return m_y;
  }
  static int mouse_flag(void){
    return m_flag;
  }
private:
  static void cv_on_mouse(int a_event, int a_x, int a_y, int a_flag, void *){
    m_event = a_event;
    m_x = a_x;
    m_y = a_y;
    m_flag = a_flag;
  }
  static int m_event;
  static int m_x;
  static int m_y;
  static int m_flag;
};
int Mouse::m_event;
int Mouse::m_x;
int Mouse::m_y;
int Mouse::m_flag;

static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename){
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());
  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);
  return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename){
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

//global
ros::Publisher create_flag;
std_msgs::Int16 flag;
tf2_ros::Buffer* tf_buffer;
cv::Ptr<cv::linemod::Detector> detector;
std::string filename;
int num_classes = 0;
int matching_threshold = 90;
int pose_flag;

void updateCreateTemplateCb(const std_msgs::Int16 msg){
  pose_flag = msg.data;
}

void callback(const sensor_msgs::Image::ConstPtr& rgb_image,
              const sensor_msgs::Image::ConstPtr& depth_image,
              const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info,
              const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info){
  bool show_match_result = true;
  cv::Size roi_size(200,200);
  int num_modalities = (int)detector->getModalities().size();
  cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  cv::Mat color = cv_rgb->image;
  cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat depth_m =  cv_depth->image;
  cv::Mat depth;
  // cv::imshow("depth_m", depth_m);
  depth_m.convertTo(depth, CV_16UC1, 1000.0);

  std::vector<cv::Mat> sources;
  sources.push_back(color);
  cv::Mat display = color.clone();

  cv::Point mouse(Mouse::x(), Mouse::y());
  int event = Mouse::event();
  int mouse_event_flag = Mouse::mouse_flag();
  cv::Point roi_offset(roi_size.width / 2, roi_size.height / 2);
  cv::Point pt1 = mouse - roi_offset; // top left
  cv::Point pt2 = mouse + roi_offset; // bottom right
  std::vector<CvPoint> chain(4);
  chain[0] = pt1;
  chain[1] = cv::Point(pt2.x, pt1.y);
  chain[2] = pt2;
  chain[3] = cv::Point(pt1.x, pt2.y);

  pose_flag = 0;

  int classes_index = num_classes-TF_OFFSET;

  if((classes_index) % SCALE_CHANGE == 0 && classes_index != 0){
    printf("chage camera distance\n");
    cv::waitKey(0);
    pose_flag = 1;
  }

  cv::Mat canny_img;
  cv::Mat gray_conf = color.clone();
  cv::cvtColor(color, gray_conf , CV_BGR2GRAY);
  cv::Canny(gray_conf, canny_img, CANNY_VAL1, CANNY_VAL2);
  //cv::imshow("canny_img", canny_img);

  // if (event == CV_EVENT_RBUTTONDOWN) {
  if (mouse_event_flag == CV_EVENT_FLAG_RBUTTON && num_classes <= SCALE_CHANGE) {
    cv::Mat mask;
    cv::Mat gray = color.clone();
    cv::cvtColor(color, gray , CV_BGR2GRAY);
    cv::Canny(gray, mask, CANNY_VAL1, CANNY_VAL2);
    cv::rectangle(mask, cv::Point(0,0), cv::Point(mask.cols,Mouse::y()-roi_size.height/2), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(0,Mouse::y()+roi_size.height/2), cv::Point(mask.cols,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(0,0), cv::Point(Mouse::x()-roi_size.width/2,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(Mouse::x()+roi_size.width/2,0), cv::Point(mask.cols,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);
    //cv::imshow("mask", mask);

    geometry_msgs::TransformStamped transform_stamped;
    try{
      transform_stamped = tf_buffer->lookupTransform("linemod_camera", "linemod_template", ros::Time(0));
    } catch(tf2::TransformException &ex) {
      ROS_WARN("tf2 error: %s", ex.what());
    }
    
    double qx = transform_stamped.transform.rotation.x;
    double qy = transform_stamped.transform.rotation.y;
    double qz = transform_stamped.transform.rotation.z;
    double qw = transform_stamped.transform.rotation.w;
    // std::cout << qx << ", " << qy << ", " << qz << ", " << qw << std::endl;

    std::string class_id = cv::format("%d,%lf,%lf,%lf,%lf", classes_index, qx, qy, qz, qw);

    cv::Rect bb;
    int template_id;
    if(num_classes >= 5){
      template_id = detector->addTemplate(sources, class_id, mask, &bb);
      if (template_id != -1){
	// printf("*** Added template ***\n");
      }
    }

    flag.data = pose_flag;
    create_flag.publish(flag);
    ++num_classes;
   }

  cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);
  cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 1);

  std::vector<cv::linemod::Match> matches;
  std::vector<std::string> class_ids;
  std::vector<cv::Mat> quantized_images;
  detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);
  int classes_visited = 0;
  std::set<std::string> visited;

  for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i){
    cv::linemod::Match m = matches[0];

    if (visited.insert(m.class_id).second){
      ++classes_visited;

      if (show_match_result){
        // printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
               // m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
	std::cout << "classes_index, qx, qy, qz, qw (object pose on linemod_camera coordinate)" << std::endl;
	std::cout << m.class_id.c_str() << std::endl;
      }
      const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
      drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));
      int similarity = (int)m.similarity;
      cv::putText(display, std::string(std::to_string(similarity))+"%",
		  cv::Point2i(m.x, m.y), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
    }
  }

  printf("-----------------------------------------------------\n");
  if (show_match_result && matches.empty())
    printf("No matches found...\n");
  cv::imshow("color", display);
  cv::FileStorage fs;
  char key = (char)cvWaitKey(10);

  switch (key){
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
  cv::waitKey(50);
}

void drawResponse(const std::vector<cv::linemod::Template>& templates, int num_modalities, cv::Mat& dst, cv::Point offset, int T){
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 255, 0), CV_RGB(255, 140, 0), CV_RGB(255, 0, 0) };
  for (int m = 0; m < num_modalities; ++m){
    cv::Scalar color = COLORS[m];
    for (int i = 0; i < (int)templates[m].features.size(); ++i) {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T / 2, color);
    }
  }
}

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > SyncPolicy;

int main(int argc, char** argv){
  ros::init(argc, argv, "linemod_rgb");
  cv::namedWindow("color");
  cv::resizeWindow("color" ,WINDOW_WIDTH,WINDOW_HEIGHT);
  Mouse::start("color");

  if (argc == 1){
    filename = "./gazebo_linemod_templates_shaft_v.yml";
    detector = cv::linemod::getDefaultLINE();
  }else{
    detector = readLinemod(argv[1]);
    std::vector<std::string> ids = detector->classIds();
    num_classes = detector->numClasses();
    if (!ids.empty()){
      printf("Class ids:\n");
      std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    }
  }

  tf2_ros::Buffer tf_buffer_ptr;
  tf_buffer = &tf_buffer_ptr;
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_image;
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_image;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgb_camera_info;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_camera_info;
  sub_rgb_image.subscribe(nh, "/camera/rgb/image_raw_gazebo", 1);
  sub_depth_image.subscribe(nh, "/camera/depth/image_raw_gazebo", 1);
  sub_rgb_camera_info.subscribe(nh, "/camera/rgb/camera_info_gazebo", 1);
  sub_depth_camera_info.subscribe(nh, "/camera/depth/camera_info_gazebo", 1);
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
  sync_->connectInput(sub_rgb_image, sub_depth_image, sub_rgb_camera_info, sub_depth_camera_info);
  sync_->registerCallback(callback);

  ros::Subscriber pose_sub = nh.subscribe("poseFrag", 1000, updateCreateTemplateCb);
  create_flag = nh.advertise<std_msgs::Int16>("createFrag", 1000);

  ros::Duration(1.0).sleep();
  ros::spin();
  return 0;
}
