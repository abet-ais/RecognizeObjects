// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, AIS Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// based on opencv sample and JSK lab linemod.cpp
// added ros interfaces
// added 2d proccecing

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose2D.h>
#include "/home/ais/ais-project/catkin_ws/src/devel/abe/abe_linemod_pkg/srv/global_pos.srv"
#include "/home/ais/ais-project/catkin_ws/src/devel/abe/abe_linemod_pkg/srv/global_id.srv"

// linemod has been moved to opencv_contrib
// http://code.opencv.org/projects/opencv/repository/revisions/1ad9827fc4ede1b9c42515569fcc5d8d1106a4ea
#if CV_MAJOR_VERSION < 3

// Function prototypes
std::string updatePartsName(cv::linemod::Match m);
int updateDeg(cv::linemod::Match m);

int detectCircle(const std::vector<cv::linemod::Template>& templates, cv::Mat& hough_img, cv::linemod::Match m);
void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f);
std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
					int num_modalities, cv::Point offset, cv::Size size,
					cv::Mat& mask, cv::Mat& dst);
void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
			  int num_modalities, cv::Point offset, cv::Size size,
			  cv::Mat& dst);
void drawResponse(const std::vector<cv::linemod::Template>& templates,
		    int num_modalities, cv::Mat& dst, cv::Point offset, int T);
cv::Mat displayQuantized(const cv::Mat& quantized);
bool init_pos_response(abe_linemod_pkg::init_pos::Request &req, abe_linemod_pkg::init_pos::Response &res);
bool id_receive(abe_linemod_pkg::ID::Request &req, abe_linemod_pkg::ID::Response &res);


// Copy of cv_mouse from cv_utilities
class Mouse
{
public:
static void start(const std::string& a_img_name)
{
cvSetMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
}
static int event(void)
{
int l_event = m_event;
m_event = -1;
return l_event;
}
static int x(void)
{
return m_x;
}
static int y(void)
{
return m_y;
}

private:
static void cv_on_mouse(int a_event, int a_x, int a_y, int, void *)
{
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

static void help()
{
printf("Usage: openni_demo [templates.yml]\n\n"
"Place your object on a planar, featureless surface. With the mouse,\n"
"frame it in the 'color' window and right click to learn a first template.\n"
"Then press 'l' to enter online learning mode, and move the camera around.\n"
"When the match score falls between 90-95%% the demo will add a new template.\n\n"
"Keys:\n"
"\t h   -- This help page\n"
"\t l   -- Toggle online learning\n"
"\t m   -- Toggle printing match result\n"
"\t t   -- Toggle printing timings\n"
"\t w   -- Write learned templates to disk\n"
"\t [ ] -- Adjust matching threshold: '[' down,  ']' up\n"
"\t q   -- Quit\n\n");
}

// Adapted from cv_timer in cv_utilities
class Timer
{
public:
Timer() : start_(0), time_(0) {}

  void start()
  {
start_ = cv::getTickCount();
}

    void stop()
    {
CV_Assert(start_ != 0);
int64 end = cv::getTickCount();
time_ += end - start_;
start_ = 0;
}

  double time()
  {
    double ret = time_ / cv::getTickFrequency();
    time_ = 0;
    return ret;
  }

private:
  int64 start_, time_;
};

// Functions to store detector and templates in single XML/YAML file
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);

  std::vector<std::string> ids = detector->classIds();
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i)
    {
      fs << "{";
      detector->writeClass(ids[i], fs);
      fs << "}"; // current class
    }
  fs << "]"; // classes
}

// global
cv::Ptr<cv::linemod::Detector> detector;
std::string filename;
int num_classes = 0;
int matching_threshold = 80;
std::string template_path = "/home/ais/ais-project/catkin_ws/src/devel/abe/abe_linemod_pkg/wrc_templates/";
ros::Publisher obj_pose_pub_;
geometry_msgs::Pose2D obj_pose_;

long int ID=0;
std::string name;
int gx[10];
int gy[10];
int set1=0;
int set2=0;
int similarity=0;

void callback(const sensor_msgs::ImageConstPtr& rgb_image)              
{
  if(ID<1){
    set1=0;
  }
  if(set1==1){
	
    switch(ID){
    case 1:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/cup_w.yml");
      break;
    case 2:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/saucer.yml");
      break;
    case 3:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/fork.yml");
      break;	
    case 4:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/knife.yml");
      break;
    case 5:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/spoon.yml");
      break;
    case 6:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/plastic_tray.yml");
      break;
    case 7:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/paper_tray.yml");
      break;
    case 8:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/pitcher.yml");
      break;
    case 9:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/power_strip.yml");
      break;
    case 10:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/.yml");
      break;
    case 11:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/ac.yml");
      break;
    case 12:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/usb.yml");
      break;
    case 13:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/roll_holder.yml");
      break;
    case 14:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/paper_towl.yml");
      break;
    case 15:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/soarting_board.yml");
      break;
    case 16:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/triangle_peg.yml");
      break;
    case 17:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/circle_peg.yml");
      break;
    case 18:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/square_peg.yml");
      break;
    case 19:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/rectangle_peg.yml");
      break;
    case 20:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/pentagon_peg.yml");
      break;
    case 21:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/smoothfoam.yml");
      break;
    case 22:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/nail.yml");
      break;
    case 23:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/hammer.yml");
      break;
    case 24:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/straw.yml");
      break;
    case 25:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/cup_with_lid");
      break;
    case 26:
      detector = readLinemod("/home/tajima/choreonoid-1.5.0/ext/graspPlugin/WRCInterfaceKnz/catkin_ws/src/WRCVision/abe_linemod_pkg/wrc_templates/vial.yml");
      break;
    }

    std::vector<std::string> ids = detector->classIds();
    num_classes = detector->numClasses();
    /*std::cout<<"Loaded "<<&name;
      printf("with %d classes and %d templates\n",
      num_classes, detector->numTemplates());*/
    if (!ids.empty())
      {
	//printf("Class ids:\n");
	//std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
      }

    bool show_match_result = true;
    bool show_timings = false;
    bool learn_online = false;  
    obj_pose_.x = 0.0;
    obj_pose_.y = 0.0; 
    obj_pose_.theta = 0.0;
    /// @todo Keys for changing these?
    // cv::Size roi_size(200, 200);
    cv::Size roi_size(300, 300);
    int learning_lower_bound = 90;
    int learning_upper_bound = 95;
    // Timers
    Timer extract_timer;
    Timer match_timer;
    int num_modalities = (int)detector->getModalities().size();
    cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
    cv::Mat color = cv_rgb->image;
    cv::Mat depth = cv_rgb->image;
    double focal_length = 400.0;
    std::vector<cv::Mat> sources;
    sources.push_back(color);
    cv::Mat display = color.clone();

    if (!learn_online)
      {
	cv::Point mouse(Mouse::x(), Mouse::y());
	int event = Mouse::event();

	// Compute ROI centered on current mouse location
	cv::Point roi_offset(roi_size.width / 2, roi_size.height / 2);
	cv::Point pt1 = mouse - roi_offset; // top left
	cv::Point pt2 = mouse + roi_offset; // bottom right
    
	if (event == CV_EVENT_RBUTTONDOWN)
	  {
	    // Compute object mask by subtracting the plane within the ROI
	    std::vector<CvPoint> chain(4);
	    chain[0] = pt1;
	    chain[1] = cv::Point(pt2.x, pt1.y);
	    chain[2] = pt2;
	    chain[3] = cv::Point(pt1.x, pt2.y);
	    cv::Mat mask;
	    subtractPlane(depth, mask, chain, focal_length);
	    cv::Mat gray = color.clone();
	    cv::cvtColor(color, gray , CV_BGR2GRAY);
	    // cv::Canny(gray, mask, 50, 200);
	    // cv::threshold(gray, mask, 0, 255, CV_THRESH_BINARY | cv::THRESH_OTSU);
	    cv::threshold(gray, mask, 200, 255, CV_THRESH_BINARY);
	    cv::rectangle(mask, cv::Point(0,0), cv::Point(color.cols,Mouse::y()-roi_size.height/2), cv::Scalar(0,0,0), -1, CV_AA);
	    cv::rectangle(mask, cv::Point(0,Mouse::y()+roi_size.height/2), cv::Point(color.cols,color.rows), cv::Scalar(0,0,0), -1, CV_AA);
	    cv::rectangle(mask, cv::Point(0,0), cv::Point(Mouse::x()-roi_size.width/2,color.rows), cv::Scalar(0,0,0), -1, CV_AA);
	    cv::rectangle(mask, cv::Point(Mouse::x()+roi_size.width/2,0), cv::Point(color.cols,color.rows), cv::Scalar(0,0,0), -1, CV_AA);

	    cv::Mat tmp_color = color.clone();
	    std::vector<cv::Mat> tmp_source;
	    tmp_source.push_back(tmp_color);

	    char parts_name[50];
	    printf("input parts name -> \n");
	    scanf("%s",parts_name);
      
	    for (int i=0; i<36; i++){
	      // Extract template
	      int deg = i*10;
	      std::string class_id = cv::format("%s,%d", parts_name, deg);
	      cv::Rect bb;
	      extract_timer.start();
	      int template_id = detector->addTemplate(tmp_source, class_id, mask, &bb);
	      extract_timer.stop();

	      if (template_id != -1){
		printf("*** Added template (id %d) for new object class %d***\n", template_id, num_classes);
		// printf("Extracted at (%d, %d) size %dx%d\n", bb.x, bb.y, bb.width, bb.height);
	      }
	      float angle = 10;
	      float scale = 1;
	      const cv::Mat affine_matrix = cv::getRotationMatrix2D(mouse, angle, scale );
	      cv::warpAffine(mask, mask, affine_matrix, mask.size());
	      cv::warpAffine(tmp_color, tmp_color, affine_matrix, mask.size());
	      cv::imshow("mask", mask);
	      ++num_classes;
	    }
	  }

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
    printf("%d\n",(int)matching_threshold);
  
    for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i)
      {
	cv::linemod::Match m = matches[i];
	const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
	std::string parts_name = updatePartsName(m);
	parts_name.pop_back();
	int deg = updateDeg(m);
	obj_pose_.theta = deg;

	if(parts_name == "female_connector3" || parts_name == "female_connector2"){
	  cv::Mat hough_img = color.clone();
	  int circle_num =  detectCircle(templates, hough_img, m);
	  if(circle_num < 1){
	    parts_name = "female_connector2";
	  } else {
	    parts_name = "female_connector3";
	  }
	}

	if (visited.insert(m.class_id).second){
	  ++classes_visited;
	  if (show_match_result){
	    printf("Similarity:%5.1f%%; x:%3d; y:%3d; %s; %d;\n",
		   m.similarity, m.x, m.y, parts_name.c_str(), deg);
	  }
	  // Draw matching template
	  drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));
	  similarity = (int)m.similarity; 
	  std::string result_text = std::string(std::to_string(similarity))+"%"+" "+(std::to_string(deg))+"deg";
	  cv::putText(display, result_text,
		      cv::Point2i(m.x, m.y), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(100,200, 0), 1);
	  cv::putText(display, parts_name,
		      cv::Point2i(m.x, m.y-20), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(100,200, 0), 1);
	  obj_pose_.x = matches[0].x;
	  obj_pose_.y = matches[0].y;
	  obj_pose_pub_.publish(obj_pose_);

	  //cv::putText(display, result_text,
	  //cv::Point2i(m.x, m.y), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0, 0), 1);
	  //      cv::putText(display, parts_name,
	  //		  cv::Point2i(m.x, m.y-20), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0, 0), 1);

	  if (learn_online == true)
	    {
	      /// @todo Online learning possibly broken by new gradient feature extraction,
	      /// which assumes an accurate object outline.

	      // Compute masks based on convex hull of matched template
	      cv::Mat color_mask, depth_mask;
	      std::vector<CvPoint> chain = maskFromTemplate(templates, num_modalities,
							    cv::Point(m.x, m.y), color.size(),
							    color_mask, display);
	      subtractPlane(depth, depth_mask, chain, focal_length);

	      cv::imshow("mask", depth_mask);

	      // If pretty sure (but not TOO sure), add new template
	      if (learning_lower_bound < m.similarity && m.similarity < learning_upper_bound)
		{
		  extract_timer.start();
		  int template_id = detector->addTemplate(sources, m.class_id, depth_mask);
		  extract_timer.stop();
		  if (template_id != -1)
		    {
		      printf("*** Added template (id %d) for existing object class %s***\n",
			     template_id, m.class_id.c_str());
		    }
		}
	    }
	}
      }

    if (show_match_result && matches.empty())
      printf("No matches found...\n");
    if (show_timings)
      {
	printf("Training: %.2fs\n", extract_timer.time());
	printf("Matching: %.2fs\n", match_timer.time());
      }
    if (show_match_result || show_timings)
      printf("------------------------------------------------------------\n");
    cv::namedWindow("color", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    cv::imshow("color", display);
    cv::imshow("normals", quantized_images[1]);

    cv::FileStorage fs;
    char key = (char)cvWaitKey(10);
    // if( key == 'q' )
    //   break;

    switch (key)
      {
      case 'h':
	help();
	break;
      case 'm':
	// toggle printing match result
	show_match_result = !show_match_result;
	printf("Show match result %s\n", show_match_result ? "ON" : "OFF");
	break;
      case 't':
	// toggle printing timings
	show_timings = !show_timings;
	printf("Show timings %s\n", show_timings ? "ON" : "OFF");
	break;
      case 'l':
	// toggle online learning
	learn_online = !learn_online;
	printf("Online learning %s\n", learn_online ? "ON" : "OFF");
	break;
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
	std::cout << "input name of template file name (.yml format) ->" << std::endl;
	std::cin >> filename;
	filename = template_path + filename;
	writeLinemod(detector, filename);
	printf("Wrote detector and templates to %s\n", filename.c_str());
	break;
      default:
	;
      }
  }

#endif
  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "linemod");
    ROS_ERROR("linemod has been moved to opencv_contrib in OpenCV3");
#if CV_MAJOR_VERSION < 3
    // Various settings and flags
    help();
    cv::namedWindow("color");
    cv::namedWindow("normals");
    Mouse::start("color");

    // register callback
    ros::NodeHandle nh;
    ros::ServiceServer ros_id_server = nh.advertiseService("ID_srv",id_receive);
    ros::ServiceServer ros_pose_server =nh.advertiseService("init_pos_srv", init_pos_response);
    ros::Subscriber sub_rgb_image;
    sub_rgb_image = nh.subscribe("/camera/rgb/image_rect_color",1, callback);
    obj_pose_pub_ = nh.advertise<geometry_msgs::Pose2D>("obj_pose", 1);

#endif
    ros::spin();
    return 0;
  }

#if CV_MAJOR_VERSION < 3

  bool id_receive(abe_linemod_pkg::ID::Request &req, abe_linemod_pkg::ID::Response &res)
  {	
    ID=req.id;
    printf("get ID=%ld\n",ID);
    set1=1;
    return true;
  }

  bool init_pos_response(abe_linemod_pkg::init_pos::Request &req, abe_linemod_pkg::init_pos::Response &res)
  {	
    //ID=req.id;
    set1=1;
    res.x=obj_pose_.x;
    res.y=obj_pose_.y;
    res.theta=obj_pose_.theta;
    res.sim=similarity;
    //printf("get ID=%ld\n",req.id);
    printf("send x=%ld y=%ld theta=%ld similarity=%ld \n",res.x ,res.y, res.theta, res.sim);
    return true;
  }

  std::string updatePartsName(cv::linemod::Match m){
    std::string parts_name = m.class_id.c_str();
    char split = ',';
    std::string parts_result;
    auto first = parts_name.begin();
    while(first != parts_name.end()){
      auto last = first;
      while( last != parts_name.end() && *last != split)
	++last;
      parts_result = std::string(parts_name.begin(), first);
      if (last != parts_name.end())
	++last;
      first = last;
    }
    return parts_result;
  }

  int updateDeg(cv::linemod::Match m){
    std::string parts_name = m.class_id.c_str();
    char split = ',';
    std::string parts_result;
    int deg;
    auto first = parts_name.begin();
    while(first != parts_name.end()){
      auto last = first;
      while( last != parts_name.end() && *last != split)
	++last;
      parts_result = std::string(first, last);
      if (last != parts_name.end())
	++last;
      first = last;
    }
    deg = std::stoi(parts_result);
    return deg;  
  }

  int detectCircle(const std::vector<cv::linemod::Template>& templates, cv::Mat& hough_img, cv::linemod::Match m){

    cv::Mat dst_img, work_img;
    dst_img = hough_img.clone();
    cv::cvtColor(dst_img, work_img, CV_BGR2GRAY);
    cv::GaussianBlur(work_img, work_img, cv::Size(11,11), 2, 2);

    cv::rectangle(work_img, cv::Point(0,0), cv::Point(dst_img.cols,m.y), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(work_img, cv::Point(0,m.y+templates[0].height), cv::Point(dst_img.cols,dst_img.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(work_img, cv::Point(0,0), cv::Point(m.x,dst_img.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(work_img, cv::Point(m.x+templates[0].width,0), cv::Point(dst_img.cols,dst_img.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(dst_img, cv::Point(0,0), cv::Point(dst_img.cols,m.y), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(dst_img, cv::Point(0,m.y+templates[0].height), cv::Point(dst_img.cols,dst_img.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(dst_img, cv::Point(0,0), cv::Point(m.x,dst_img.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(dst_img, cv::Point(m.x+templates[0].width,0), cv::Point(dst_img.cols,dst_img.rows), cv::Scalar(0,0,0), -1, CV_AA);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(work_img, circles, CV_HOUGH_GRADIENT, 2, 25, 50, 30, 5, 13);
  
    std::vector<cv::Vec3f>::iterator it = circles.begin();
    for(; it!=circles.end(); ++it) {
      cv::Point center(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
      int radius = cv::saturate_cast<int>((*it)[2]);
      cv::circle(dst_img, center, 3, cv::Scalar(0,255,0), 2);
      cv::circle(dst_img, center, radius, cv::Scalar(0,0,255), 2);
    }
    cv::namedWindow("HoughCircles", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    cv::imshow("HoughCircles", dst_img);

    return (int)circles.size();
  }

  static void reprojectPoints(const std::vector<cv::Point3d>& proj, std::vector<cv::Point3d>& real, double f)
  {
    real.resize(proj.size());
    double f_inv = 1.0 / f;

    for (int i = 0; i < (int)proj.size(); ++i)
      {
	double Z = proj[i].z;
	real[i].x = (proj[i].x - 320.) * (f_inv * Z);
	real[i].y = (proj[i].y - 240.) * (f_inv * Z);
	real[i].z = Z;
      }
  }

  static void filterPlane(IplImage * ap_depth, std::vector<IplImage *> & a_masks, std::vector<CvPoint> & a_chain, double f)
  {
    const int l_num_cost_pts = 200;

    float l_thres = 4;

    IplImage * lp_mask = cvCreateImage(cvGetSize(ap_depth), IPL_DEPTH_8U, 1);
    cvSet(lp_mask, cvRealScalar(0));

    std::vector<CvPoint> l_chain_vector;

    float l_chain_length = 0;
    float * lp_seg_length = new float[a_chain.size()];

    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
      {
	float x_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x);
	float y_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y);
	lp_seg_length[l_i] = sqrt(x_diff*x_diff + y_diff*y_diff);
	l_chain_length += lp_seg_length[l_i];
      }
    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
      {
	if (lp_seg_length[l_i] > 0)
	  {
	    int l_cur_num = cvRound(l_num_cost_pts * lp_seg_length[l_i] / l_chain_length);
	    float l_cur_len = lp_seg_length[l_i] / l_cur_num;

	    for (int l_j = 0; l_j < l_cur_num; ++l_j)
	      {
		float l_ratio = (l_cur_len * l_j / lp_seg_length[l_i]);

		CvPoint l_pts;

		l_pts.x = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x) + a_chain[l_i].x);
		l_pts.y = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y) + a_chain[l_i].y);

		l_chain_vector.push_back(l_pts);
	      }
	  }
      }
    std::vector<cv::Point3d> lp_src_3Dpts(l_chain_vector.size());

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
      {
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

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
      {
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

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
      {
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

    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
      {
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

    for (int l_r = 0; l_r < l_h; ++l_r)
      {
	for (int l_c = 0; l_c < l_w; ++l_c)
	  {
	    lp_dst_3Dpts[l_ind].x = l_c + l_minx;
	    lp_dst_3Dpts[l_ind].y = l_r + l_miny;
	    lp_dst_3Dpts[l_ind].z = CV_IMAGE_ELEM(ap_depth, unsigned short, l_r + l_miny, l_c + l_minx);
	    ++l_ind;
	  }
      }
    reprojectPoints(lp_dst_3Dpts, lp_dst_3Dpts, f);

    l_ind = 0;

    for (int l_r = 0; l_r < l_h; ++l_r)
      {
	for (int l_c = 0; l_c < l_w; ++l_c)
	  {
	    float l_dist = (float)(l_n[0] * lp_dst_3Dpts[l_ind].x + l_n[1] * lp_dst_3Dpts[l_ind].y + lp_dst_3Dpts[l_ind].z * l_n[2] + l_n[3]);

	    ++l_ind;

	    if (CV_IMAGE_ELEM(lp_mask, unsigned char, l_r + l_miny, l_c + l_minx) != 0)
	      {
		if (fabs(l_dist) < std::max(l_thres, (l_max_dist * 2.0f)))
		  {
		    for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p)
		      {
			int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
			int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));

			CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 0;
		      }
		  }
		else
		  {
		    for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p)
		      {
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

  void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f)
  {
    mask = cv::Mat::zeros(depth.size(), CV_8U);
    std::vector<IplImage*> tmp;
    IplImage mask_ipl = mask;
    tmp.push_back(&mask_ipl);
    IplImage depth_ipl = depth;
    filterPlane(&depth_ipl, tmp, chain, f);
  }

  std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
					int num_modalities, cv::Point offset, cv::Size size,
					cv::Mat& mask, cv::Mat& dst)
  {
    templateConvexHull(templates, num_modalities, offset, size, mask);

    const int OFFSET = 30;
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), OFFSET);

    CvMemStorage * lp_storage = cvCreateMemStorage(0);
    CvTreeNodeIterator l_iterator;
    CvSeqReader l_reader;
    CvSeq * lp_contour = 0;

    cv::Mat mask_copy = mask.clone();
    IplImage mask_copy_ipl = mask_copy;
    cvFindContours(&mask_copy_ipl, lp_storage, &lp_contour, sizeof(CvContour),
		   CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    std::vector<CvPoint> l_pts1; // to use as input to cv_primesensor::filter_plane

    cvInitTreeNodeIterator(&l_iterator, lp_contour, 1);
    while ((lp_contour = (CvSeq *)cvNextTreeNode(&l_iterator)) != 0)
      {
	CvPoint l_pt0;
	cvStartReadSeq(lp_contour, &l_reader, 0);
	CV_READ_SEQ_ELEM(l_pt0, l_reader);
	l_pts1.push_back(l_pt0);

	for (int i = 0; i < lp_contour->total; ++i)
	  {
	    CvPoint l_pt1;
	    CV_READ_SEQ_ELEM(l_pt1, l_reader);
	    /// @todo Really need dst at all? Can just as well do this outside
	    cv::line(dst, l_pt0, l_pt1, CV_RGB(0, 255, 0), 2);

	    l_pt0 = l_pt1;
	    l_pts1.push_back(l_pt0);
	  }
      }
    cvReleaseMemStorage(&lp_storage);

    return l_pts1;
  }

  // Adapted from cv_show_angles
  cv::Mat displayQuantized(const cv::Mat& quantized)
  {
    cv::Mat color(quantized.size(), CV_8UC3);
    for (int r = 0; r < quantized.cols; ++r)
      {
	const uchar* quant_r = quantized.ptr(r);
	cv::Vec3b* color_r = color.ptr<cv::Vec3b>(r);

	for (int c = 0; c < quantized.rows; ++c)
	  {
	    cv::Vec3b& bgr = color_r[c];
	    switch (quant_r[c])
	      {
	      case 0:   bgr[0]=  0; bgr[1]=  0; bgr[2]=  0;    break;
	      case 1:   bgr[0]= 55; bgr[1]= 55; bgr[2]= 55;    break;
	      case 2:   bgr[0]= 80; bgr[1]= 80; bgr[2]= 80;    break;
	      case 4:   bgr[0]=105; bgr[1]=105; bgr[2]=105;    break;
	      case 8:   bgr[0]=130; bgr[1]=130; bgr[2]=130;    break;
	      case 16:  bgr[0]=155; bgr[1]=155; bgr[2]=155;    break;
	      case 32:  bgr[0]=180; bgr[1]=180; bgr[2]=180;    break;
	      case 64:  bgr[0]=205; bgr[1]=205; bgr[2]=205;    break;
	      case 128: bgr[0]=230; bgr[1]=230; bgr[2]=230;    break;
	      case 255: bgr[0]=  0; bgr[1]=  0; bgr[2]=255;    break;
	      default:  bgr[0]=  0; bgr[1]=255; bgr[2]=  0;    break;
	      }
	  }
      }

    return color;
  }

  // Adapted from cv_line_template::convex_hull
  void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
			  int num_modalities, cv::Point offset, cv::Size size,
			  cv::Mat& dst)
  {
    std::vector<cv::Point> points;
    for (int m = 0; m < num_modalities; ++m)
      {
	for (int i = 0; i < (int)templates[m].features.size(); ++i)
	  {
	    cv::linemod::Feature f = templates[m].features[i];
	    points.push_back(cv::Point(f.x, f.y) + offset);
	  }
      }

    std::vector<cv::Point> hull;
    cv::convexHull(points, hull);

    dst = cv::Mat::zeros(size, CV_8U);
    const int hull_count = (int)hull.size();
    const cv::Point* hull_pts = &hull[0];
    cv::fillPoly(dst, &hull_pts, &hull_count, 1, cv::Scalar(255));
  }

  void drawResponse(const std::vector<cv::linemod::Template>& templates,
		    int num_modalities, cv::Mat& dst, cv::Point offset, int T)
  {
    static const cv::Scalar COLORS[5] = { CV_RGB(0, 255, 0),
					  CV_RGB(0, 0, 255),
					  CV_RGB(255, 255, 0),
					  CV_RGB(255, 140, 0),
					  CV_RGB(255, 0, 0) };

    for (int m = 0; m < num_modalities; ++m)
      {
	// NOTE: Original demo recalculated max response for each feature in the TxT
	// box around it and chose the display color based on that response. Here
	// the display color just depends on the modality.
	cv::Scalar color = COLORS[m];

	for (int i = 0; i < (int)templates[m].features.size(); ++i)
	  {
	    cv::rectangle(dst, cv::Point(offset.x, offset.y), cv::Point(offset.x+templates[m].width, offset.y+templates[m].height), CV_RGB(0,200,200), 2);
	    cv::linemod::Feature f = templates[m].features[i];
	    cv::Point pt(f.x + offset.x, f.y + offset.y);
	    cv::circle(dst, pt, T, color);
	  }
      }
  }
#endif
