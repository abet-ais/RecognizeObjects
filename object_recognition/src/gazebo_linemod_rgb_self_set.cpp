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

#if CV_MAJOR_VERSION < 3
// #define SCALE_CHANGE 1158
#define SCALE_CHANGE 4381
// #define SCALE_CHANGE 2358
#define CANNY_VAL1 50
#define CANNY_VAL2 100
#define INITIAL_DIST 0.7

void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f);
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
geometry_msgs::Vector3 gazebo_pose;
geometry_msgs::Quaternion quat_gazebo_pose;
tf::TransformListener* listener_ptr;
tf2_ros::Buffer* tf_buffer;
cv::Ptr<cv::linemod::Detector> detector;
std::string filename;
int num_classes = 0;
int matching_threshold = 90;

void callback(const sensor_msgs::Image::ConstPtr& rgb_image,
              const sensor_msgs::Image::ConstPtr& depth_image,
              const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info,
              const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info){
  bool show_match_result = true;
  /// @todo Keys for changing these?
  cv::Size roi_size(400, 400);
  int num_modalities = (int)detector->getModalities().size();
  cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  cv::Mat color = cv_rgb->image;
  cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  // cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  //cv::Mat depth = 1000 * cv_depth->image;
  cv::Mat depth_m =  cv_depth->image;
  cv::Mat depth;
  depth_m.convertTo(depth, CV_16UC1, 1000.0);
  // Capture next color/depth pair
  // capture.grab();
  // capture.retrieve(depth, CV_CAP_OPENNI_DEPTH_MAP);
  // capture.retrieve(color, CV_CAP_OPENNI_BGR_IMAGE);
  //double focal_length = capture.get(CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
  double focal_length = depth_camera_info->K[0]; // fx

  std::vector<cv::Mat> sources;
  sources.push_back(color);
  // sources.push_back(depth/1000.0);
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

  cv::Mat canny_img;
  cv::Mat gray_conf = color.clone();
  cv::cvtColor(color, gray_conf , CV_BGR2GRAY);
  cv::Canny(gray_conf, canny_img, CANNY_VAL1, CANNY_VAL2);
  cv::imshow("canny_img", canny_img);

  if (event == CV_EVENT_RBUTTONDOWN) {
    cv::Mat mask;
    cv::Mat gray = color.clone();
    cv::cvtColor(color, gray , CV_BGR2GRAY);
    cv::Canny(gray, mask, CANNY_VAL1, CANNY_VAL2);
    // cv::threshold(gray, mask, 0, 255, CV_THRESH_BINARY | cv::THRESH_OTSU);
    // cv::threshold(gray, mask, 100, 255, CV_THRESH_BINARY);
    cv::rectangle(mask, cv::Point(0,0), cv::Point(mask.cols,Mouse::y()-roi_size.height/2), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(0,Mouse::y()+roi_size.height/2), cv::Point(mask.cols,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(0,0), cv::Point(Mouse::x()-roi_size.width/2,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(Mouse::x()+roi_size.width/2,0), cv::Point(mask.cols,mask.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::imshow("mask", mask);

    geometry_msgs::TransformStamped transform_stamped;
    try{
      transform_stamped = tf_buffer->lookupTransform("linemod_camera", "linemod_template", ros::Time(0));
      // transform_stamped = tf_buffer->lookupTransform("linemod_camera", "linemod_template", ros::Time(0));
    } catch(tf2::TransformException &ex) {
      ROS_WARN("tf2 error: %s", ex.what());
    }
    
    double qx = transform_stamped.transform.rotation.x;
    double qy = transform_stamped.transform.rotation.y;
    double qz = transform_stamped.transform.rotation.z;
    double qw = transform_stamped.transform.rotation.w;
    std::cout << qx << ", "
	      << qy << ", "
	      << qz << ", "
	      << qw << std::endl;

    std::string class_id = cv::format("%d,%lf,%lf,%lf,%lf", num_classes, qx, qy, qz, qw);

    cv::Rect bb;
    int template_id;

    template_id = detector->addTemplate(sources, class_id, mask, &bb);
    if (template_id != -1){
      printf("*** Added template (id %d) for new object class %d***\n", template_id, num_classes);
    }
    ++num_classes;
   }

  cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);
  cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 1);

  std::vector<cv::linemod::Match> matches;
  std::vector<std::string> class_ids;
  std::vector<cv::Mat> quantized_images;
  detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);
  // cv::imshow("normals", depth*1000);
  int classes_visited = 0;
  std::set<std::string> visited;

  for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i){
    cv::linemod::Match m = matches[0];

    if (visited.insert(m.class_id).second){
      ++classes_visited;

      if (show_match_result){
        // printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
               // m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
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
  // cv::waitKey(500);
  cv::waitKey(50);
}

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > SyncPolicy;

#endif
int main(int argc, char** argv){
  ros::init(argc, argv, "linemod");
  ROS_ERROR("linemod has been moved to opencv_contrib in OpenCV3");
#if CV_MAJOR_VERSION < 3
  cv::namedWindow("color");
  cv::namedWindow("normals");
  Mouse::start("color");
  if (argc == 1){
    filename = "./gazebo_linemod_templates.yml";
    // detector = cv::linemod::getDefaultLINEMOD();
    detector = cv::linemod::getDefaultLINE();
  }else{
    detector = readLinemod(argv[1]);
    std::vector<std::string> ids = detector->classIds();
    num_classes = detector->numClasses();
    printf("Loaded %s with %d classes and %d templates\n", argv[1], num_classes, detector->numTemplates());
    if (!ids.empty()){
      printf("Class ids:\n");
      std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    }
  }
  // tf::TransformListener listener;
  // listener_ptr = &listener;
  tf2_ros::Buffer tf_buffer_ptr;
  tf_buffer = &tf_buffer_ptr;
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_image;
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_image;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgb_camera_info;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_camera_info;
  sub_rgb_image.subscribe(nh, "/camera/rgb/image_raw", 1);
  sub_depth_image.subscribe(nh, "/camera/depth/image_raw", 1);
  sub_rgb_camera_info.subscribe(nh, "/camera/rgb/camera_info", 1);
  sub_depth_camera_info.subscribe(nh, "/camera/depth/camera_info", 1);
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
  sync_->connectInput(sub_rgb_image, sub_depth_image, sub_rgb_camera_info, sub_depth_camera_info);
  sync_->registerCallback(callback);
#endif

  ros::Duration(1.0).sleep();
  ros::spin();
  return 0;
}

#if CV_MAJOR_VERSION < 3
static void reprojectPoints(const std::vector<cv::Point3d>& proj, std::vector<cv::Point3d>& real, double f){
  real.resize(proj.size());
  double f_inv = 1.0 / f;
  for (int i = 0; i < (int)proj.size(); ++i) {
    double Z = proj[i].z;
    real[i].x = (proj[i].x - 320.) * (f_inv * Z);
    real[i].y = (proj[i].y - 240.) * (f_inv * Z);
    real[i].z = Z;
  }
}

static void filterPlane(IplImage * ap_depth, std::vector<IplImage *> & a_masks, std::vector<CvPoint> & a_chain, double f){
  const int l_num_cost_pts = 200;
  float l_thres = 4;
  IplImage * lp_mask = cvCreateImage(cvGetSize(ap_depth), IPL_DEPTH_8U, 1);
  cvSet(lp_mask, cvRealScalar(0));

  std::vector<CvPoint> l_chain_vector;
  float l_chain_length = 0;
  float * lp_seg_length = new float[a_chain.size()];

  for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i){
    float x_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x);
    float y_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y);
    lp_seg_length[l_i] = sqrt(x_diff*x_diff + y_diff*y_diff);
    l_chain_length += lp_seg_length[l_i];
  }
  for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i){
    if (lp_seg_length[l_i] > 0) {
      int l_cur_num = cvRound(l_num_cost_pts * lp_seg_length[l_i] / l_chain_length);
      float l_cur_len = lp_seg_length[l_i] / l_cur_num;
      for (int l_j = 0; l_j < l_cur_num; ++l_j) {
        float l_ratio = (l_cur_len * l_j / lp_seg_length[l_i]);
        CvPoint l_pts;
        l_pts.x = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x) + a_chain[l_i].x);
        l_pts.y = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y) + a_chain[l_i].y);
        l_chain_vector.push_back(l_pts);
      }
    }
  }
  std::vector<cv::Point3d> lp_src_3Dpts(l_chain_vector.size());
  for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i){
    lp_src_3Dpts[l_i].x = l_chain_vector[l_i].x;
    lp_src_3Dpts[l_i].y = l_chain_vector[l_i].y;
    lp_src_3Dpts[l_i].z = CV_IMAGE_ELEM(ap_depth, unsigned short, cvRound(lp_src_3Dpts[l_i].y), cvRound(lp_src_3Dpts[l_i].x));
    //CV_IMAGE_ELEM(lp_mask,unsigned char,(int)lp_src_3Dpts[l_i].Y,(int)lp_src_3Dpts[l_i].X)=255;
  }
  //cv_show_image(lp_mask,"hallo2");
  reprojectPoints(lp_src_3Dpts, lp_src_3Dpts, f);
  CvMat * lp_pts = cvCreateMat((int)l_chain_vector.size(), 4, CV_32F);
  CvMat * lp_v = cvCreateMat(4, 4, CV_32F);
  CvMat * lp_w = cvCreateMat(4, 1, CV_32F);
  for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i){
    CV_MAT_ELEM(*lp_pts, float, l_i, 0) = (float)lp_src_3Dpts[l_i].x;
    CV_MAT_ELEM(*lp_pts, float, l_i, 1) = (float)lp_src_3Dpts[l_i].y;
    CV_MAT_ELEM(*lp_pts, float, l_i, 2) = (float)lp_src_3Dpts[l_i].z;
    CV_MAT_ELEM(*lp_pts, float, l_i, 3) = 1.0f;
  }
  cvSVD(lp_pts, lp_w, 0, lp_v);
  float l_n[4] = {CV_MAT_ELEM(*lp_v, float, 0, 3),
                  CV_MAT_ELEM(*lp_v, float, 1, 3),
                  CV_MAT_ELEM(*lp_v, float, 2, 3),
                  CV_MAT_ELEM(*lp_v, float, 3, 3)};
  float l_norm = sqrt(l_n[0] * l_n[0] + l_n[1] * l_n[1] + l_n[2] * l_n[2]);
  l_n[0] /= l_norm;
  l_n[1] /= l_norm;
  l_n[2] /= l_norm;
  l_n[3] /= l_norm;
  float l_max_dist = 0;
  for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i){
    float l_dist =  l_n[0] * CV_MAT_ELEM(*lp_pts, float, l_i, 0) +
      l_n[1] * CV_MAT_ELEM(*lp_pts, float, l_i, 1) +
      l_n[2] * CV_MAT_ELEM(*lp_pts, float, l_i, 2) +
      l_n[3] * CV_MAT_ELEM(*lp_pts, float, l_i, 3);
    if (fabs(l_dist) > l_max_dist)
      l_max_dist = l_dist;
  }
  //std::cerr << "plane: " << l_n[0] << ";" << l_n[1] << ";" << l_n[2] << ";" << l_n[3] << " maxdist: " << l_max_dist << " end" << std::endl;
  int l_minx = ap_depth->width;
  int l_miny = ap_depth->height;
  int l_maxx = 0;
  int l_maxy = 0;
  for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i){
    l_minx = std::min(l_minx, a_chain[l_i].x);
    l_miny = std::min(l_miny, a_chain[l_i].y);
    l_maxx = std::max(l_maxx, a_chain[l_i].x);
    l_maxy = std::max(l_maxy, a_chain[l_i].y);
  }
  int l_w = l_maxx - l_minx + 1;
  int l_h = l_maxy - l_miny + 1;
  int l_nn = (int)a_chain.size();
  CvPoint * lp_chain = new CvPoint[l_nn];
  for (int l_i = 0; l_i < l_nn; ++l_i)
    lp_chain[l_i] = a_chain[l_i];
  cvFillPoly(lp_mask, &lp_chain, &l_nn, 1, cvScalar(255, 255, 255));
  delete[] lp_chain;
  //cv_show_image(lp_mask,"hallo1");
  std::vector<cv::Point3d> lp_dst_3Dpts(l_h * l_w);
  int l_ind = 0;
  for (int l_r = 0; l_r < l_h; ++l_r){
    for (int l_c = 0; l_c < l_w; ++l_c){
      lp_dst_3Dpts[l_ind].x = l_c + l_minx;
      lp_dst_3Dpts[l_ind].y = l_r + l_miny;
      lp_dst_3Dpts[l_ind].z = CV_IMAGE_ELEM(ap_depth, unsigned short, l_r + l_miny, l_c + l_minx);
      ++l_ind;
    }
  }
  reprojectPoints(lp_dst_3Dpts, lp_dst_3Dpts, f);
  l_ind = 0;
  for (int l_r = 0; l_r < l_h; ++l_r){
    for (int l_c = 0; l_c < l_w; ++l_c){
      float l_dist = (float)(l_n[0] * lp_dst_3Dpts[l_ind].x + l_n[1] * lp_dst_3Dpts[l_ind].y + lp_dst_3Dpts[l_ind].z * l_n[2] + l_n[3]);
      ++l_ind;
      if (CV_IMAGE_ELEM(lp_mask, unsigned char, l_r + l_miny, l_c + l_minx) != 0){
        if (fabs(l_dist) < std::max(l_thres, (l_max_dist * 2.0f))){
          for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p){
            int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
            int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));
            CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 0;
          }
        }else{
          for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p){
            int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
            int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));
            CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 255;
          }
        }
      }
    }
  }
  cvReleaseImage(&lp_mask);
  cvReleaseMat(&lp_pts);
  cvReleaseMat(&lp_w);
  cvReleaseMat(&lp_v);
}

void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f){
  mask = cv::Mat::zeros(depth.size(), CV_8U);
  std::vector<IplImage*> tmp;
  IplImage mask_ipl = mask;
  tmp.push_back(&mask_ipl);
  IplImage depth_ipl = depth;
  filterPlane(&depth_ipl, tmp, chain, f);
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
#endif
