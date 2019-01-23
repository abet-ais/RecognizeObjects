//arg[1] パーツの名前をいれてください(class_idとymlファイルの名前になります)
//arg[2] バウンディングボックスのサイズ

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
#include "abe_sandbox/createTemp_switch.h"


#define SCALE_CHANGE 7000
#define CANNY_VAL1 20
#define CANNY_VAL2 40
#define INITIAL_DIST 0.7
#define TF_OFFSET 5
#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080


tf2_ros::Buffer* tf_buffer;
cv::Ptr<cv::linemod::Detector> detector;
ros::ServiceClient switch_client;
std::string filename;
int num_classes = 0;
int matching_threshold = 90;
int bb_roi_size;
std::string class_id; 



class Mouse{
public:
  static void start(const std::string& a_img_name){
    cvSetMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
  }
  static int event(void){
    int l_event = m_event;
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



void callback(const sensor_msgs::Image::ConstPtr& rgb_image,
              const sensor_msgs::Image::ConstPtr& depth_image,
              const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info,
              const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info){
  bool show_match_result = true;
  cv::Size roi_size(bb_roi_size,bb_roi_size);
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

  int event = Mouse::event();
  int mouse_event_flag = Mouse::mouse_flag();
  cv::Point mouse(Mouse::x(), Mouse::y());
  //  std::cout <<"x"<<mouse.x<<" y"<<mouse.y;
  cv::Point roi_offset( roi_size.width/2, roi_size.height/2 );
  // cv::Point pt1 = mouse - roi_offset;
  // cv::Point pt2 = mouse + roi_offset;
  
  int classes_index = num_classes-TF_OFFSET;

  if((classes_index) % SCALE_CHANGE == 0 && classes_index != 0){
    printf("chage camera distance\n");
    cv::waitKey(0);
    //pose_flag = 1;
  }

  cv::Mat canny_img;
  cv::Mat gray_conf = color.clone();
  cv::cvtColor(color, gray_conf , CV_BGR2GRAY);
  cv::Canny(gray_conf, canny_img, CANNY_VAL1, CANNY_VAL2);
  //cv::imshow("canny_img", canny_img);
  
  static bool mouse_event_sw = false;

  if ( (mouse_event_flag == CV_EVENT_FLAG_RBUTTON || mouse_event_sw == true) && num_classes <= SCALE_CHANGE) {
    mouse_event_sw = true;

    static abe_sandbox::createTemp_switch sw_srv;
    int x_pix = 0;
    int y_pix = 0;
    static int x_pix_p = 0;
    static int y_pix_p = 0;

    sw_srv.request.movement++;
    if(switch_client.call(sw_srv)){
      int check = sw_srv.response.check;
      x_pix = sw_srv.response.x_pixel;
      y_pix = sw_srv.response.y_pixel;
      std::cout <<" check:"<<check<<" x_p:"<<x_pix<<" y_p:"<<y_pix << std::endl;
    }else{
      std::cout << "fail" << std::endl;
      return;
    }
    
    cv::Point componentCenter(x_pix_p,y_pix_p);
    cv::Point pt1 = componentCenter - roi_offset;
    cv::Point pt2 = componentCenter + roi_offset;
    x_pix_p = x_pix;
    y_pix_p = y_pix;

    cv::circle(display,componentCenter,2,cv::Scalar(0,0,255),3);
    cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);
    cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 2);
    
    cv::Mat mask;
    cv::Mat gray = color.clone();
    cv::cvtColor(color, gray , CV_BGR2GRAY);
    cv::Canny(gray, mask, CANNY_VAL1, CANNY_VAL2);
    cv::rectangle(mask, cv::Point(0,0), cv::Point(mask.cols,pt1.y), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(0,pt2.y), cv::Point(mask.cols,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(0,0), cv::Point(pt1.x,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(pt2.x,0), cv::Point(mask.cols,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);

    printf("classes_index : %d\n",classes_index);

    geometry_msgs::TransformStamped transform_stamped;
    try{
      //lookupTransform:カメラから部品を見た時どのような回転をしているか
      transform_stamped = tf_buffer->lookupTransform("linemod_camera", "linemod_template", ros::Time(0));
    } catch(tf2::TransformException &ex) {
      ROS_WARN("tf2 error: %s", ex.what());
    }

    float  x  = transform_stamped.transform.translation.x;
    float  y  = transform_stamped.transform.translation.y;
    float  z  = transform_stamped.transform.translation.z;
    double qx = transform_stamped.transform.rotation.x;
    double qy = transform_stamped.transform.rotation.y;
    double qz = transform_stamped.transform.rotation.z;
    double qw = transform_stamped.transform.rotation.w;
    //    std::cout <<x<<" "<<y<<" "<< z <<std::endl;
    // std::cout << qx << ", " << qy << ", " << qz << ", " << qw << std::endl;
    
    cv::Rect bb; 
    int template_id;
    if(num_classes > 0){
      template_id = detector->addTemplate(sources, class_id+",0", mask, &bb);
      if(template_id != -1){
	printf("+++ added template +++\n");
      }else{
	show_match_result = false;
      }
    }
    ++num_classes;
  }

  // std::vector<cv::linemod::Match> matches;
  // std::vector<std::string> class_ids;
  // std::vector<cv::Mat> quantized_images;
  // int classes_visited = 0;
  // std::set<std::string> visited;

  // detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);

  // for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i){
  //   cv::linemod::Match m = matches[0];

  //   if (visited.insert(m.class_id).second){
  //     ++classes_visited;

  //     if (show_match_result){
  //       // printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
  //              // m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
  // 	std::cout << "classes_index, qx, qy, qz, qw (object pose on linemod_camera coordinate)" << std::endl;
  // 	std::cout << m.class_id.c_str() << std::endl;
  //     }
  //     const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
  //     drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));
  //     int similarity = (int)m.similarity;
  //     cv::putText(display, std::string(std::to_string(similarity))+"%",
  // 		  cv::Point2i(m.x, m.y), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
  //   }
  // }

  printf("-----------------------------------------------------\n");
  //  if (show_match_result && matches.empty());
  if(show_match_result == false)
    printf("No matches found...\n");
  cv::imshow("color", display);
  cv::FileStorage fs;
  char key = (char)cvWaitKey(10);

  switch (key){
  case 'w':
    // write model to disk
    writeLinemod(detector, filename);
    printf("Wrote detector and templates to %s\n", filename.c_str());
    break;
  default:
    ;
  }
}



typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
							sensor_msgs::Image, 
							sensor_msgs::CameraInfo, 
							sensor_msgs::CameraInfo > SyncPolicy;



int main(int argc, char** argv){
  ros::init(argc, argv, "createTemp_TB_gazebo");
  cv::namedWindow("color",CV_WINDOW_NORMAL);
  cv::resizeWindow("color" ,WINDOW_WIDTH,WINDOW_HEIGHT);
  Mouse::start("color");
 
  detector = cv::linemod::getDefaultLINE();

  class_id = (argv[1]);                 //実行時引数 parts name
  bb_roi_size = std::stoi(argv[2]);     //実行時引数 baunding box size
  filename = class_id + ".yml";

  ros::NodeHandle nh;
  tf2_ros::Buffer tf_buffer_ptr;
  tf_buffer = &tf_buffer_ptr;
  tf2_ros::TransformListener tf_listener(*tf_buffer);

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

  switch_client = nh.serviceClient<abe_sandbox::createTemp_switch>("sw_pose");

  ros::Duration(0.1).sleep();
  ros::spin();
  return 0;
}
