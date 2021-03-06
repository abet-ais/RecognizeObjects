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

#define BACKGROUND_FILTER 0.71
#define FRONT_FILTER 0.1
#define BB_AREA 10
#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720

#define object_num 1  //検出する物体の数

// global variable
cv::Ptr<cv::linemod::Detector> detector[object_num]= cv::linemod::getDefaultLINEMOD(); //detectorという名のクラスをポインタで定義


std::string filename;
std::vector<std::string> FILENAME;
// 使用していない→ std::string template_path = "/home/ais/ais-project/catkin_ws/src/devel/abe/abe_linemod_pkg/linemod_template/";
int num_classes[object_num] = {0};

void drawResponse(const std::vector<cv::linemod::Template>& templates,int num_modalities, cv::Mat& dst,cv::Mat& drawBordering_display, cv::Point offset, int T);
void split3dPose(cv::linemod::Match m, double& quat_x, double& quat_y, double& quat_z, double& quat_w);


/*----- class mouse ----------------------------------------------------*/
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



/*----- class Timer --------------------------------------------------------*/
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
  cv::FileStorage fs(filename, cv::FileStorage::READ);//xmlとymlファイルの読み書き
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}


void split3dPose(cv::linemod::Match m, double& quat_x, double& quat_y, double& quat_z, double& quat_w) {
  std::string class_id = m.class_id.c_str();
  auto string = class_id;

  // delimiter 区切り？
  auto separator = std::string(",");         
  auto separator_length = separator.length(); 
  // vector for store each splited string 各分割文字列を格納するベクトル

  auto list = std::vector<std::string>();

  if (separator_length == 0) {
    list.push_back(string);
  } else {
    auto offset = std::string::size_type(0);//coutの結果は0だった
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

  quat_x = std::stod(list.at(1));
  quat_y = std::stod(list.at(2));
  quat_z = std::stod(list.at(3));
  quat_w = std::stod(list.at(4));

}


void drawResponse(const std::vector<cv::linemod::Template>& templates,int num_modalities, cv::Mat& dst,cv::Mat& drawBordering_disp, cv::Point offset, int T) {
  static const cv::Scalar COLORS[5] = {CV_RGB(0, 255, 0), CV_RGB(0, 0, 255)};

  for (int m = 0; m < num_modalities; ++m) {
    cv::Scalar color = COLORS[m];
    /* 検出したオブジェクトの概形を矩形で囲う */
    cv::rectangle(dst, cv::Point(offset.x, offset.y),cv::Point(offset.x+templates[m].width, offset.y+templates[m].height),CV_RGB(255,255,255), -1,CV_AA);

    /* 検出したオブジェクトの中心に円を描く  */
    cv::Point obj_center(offset.x+templates[m].width/2, offset.y+templates[m].height/2);//テンプレートの中心をとる
    cv::circle(dst, obj_center, 2 , cv::Scalar(0,0,255), -1); //円の描画

    for (int i = 0; i < (int)templates[m].features.size(); ++i) {   
      /* 複数の小さい円で検出したオブジェクトをかたどる */
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(drawBordering_disp, pt, 3, color, -1);//引数(画像,円の中心座標,円の半径,色,?)
    }
  }
}


/*----- class rosLinemod -------------------------------------------*/
class rosLinemod {
private:
  int matching_threshold = 80;
  geometry_msgs::PoseStamped obj_pose_;
  geometry_msgs::Quaternion quat_pose;
  cv::Mat depth;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_rgb_image_;
  image_transport::Subscriber sub_depth_image_;
  ros::Publisher obj_pose_pub_;

  ros::NodeHandle nh;
  ros::Publisher obj_num_pub;

  void depthCb(const sensor_msgs::ImageConstPtr& depth_image);
  void imageCb(const sensor_msgs::ImageConstPtr& rgb_image);

public:
  rosLinemod();
  ~rosLinemod();
};

rosLinemod::rosLinemod() : nh_(), it_(nh_){//コンストラクタ
  // for xtion
  sub_rgb_image_ = it_.subscribe("/camera/rgb/image_raw",1, &rosLinemod::imageCb, this);//xtionから左のtopicが飛んできたらCbが呼ばれる
  sub_depth_image_ = it_.subscribe("/camera/depth_registered/image_raw",1, &rosLinemod::depthCb, this);
  // for realsense
  // sub_rgb_image_ = it_.subscribe("/camera/color/image_raw",1, &rosLinemod::imageCb, this);
  // sub_depth_image_ = it_.subscribe("/camera/aligned_depth_to_color/image_raw",1, &rosLinemod::depthCb, this);
  obj_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/linemod/obj_pose", 1);

  obj_pose_.pose.position.x = 0.0;   //0.0で初期化
  obj_pose_.pose.position.y = 0.0; 
  obj_pose_.pose.position.z = 0.0;
  obj_pose_.pose.orientation.x = 0.0;
  obj_pose_.pose.orientation.y = 0.0; 
  obj_pose_.pose.orientation.z = 0.0;
  obj_pose_.pose.orientation.w = 0.0;
}

rosLinemod::~rosLinemod(){}

void rosLinemod::depthCb(const sensor_msgs::ImageConstPtr& depth_image){ //rosの型をcv::Matに変換している
  cv_bridge::CvImagePtr cv_depth;
  try{
    cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depth = cv_depth->image;
}


void rosLinemod::imageCb(const sensor_msgs::ImageConstPtr& rgb_image) {
  bool show_match_result = true;
  bool show_timings = false;

  cv::Size roi_size(200, 200);

  // Timers
  Timer extract_timer;
  Timer match_timer;
  int num_modalities[object_num];
  for(int k=0; k<object_num; k++){
    num_modalities[k] = (int)detector[k]->getModalities().size();
  }

  cv_bridge::CvImagePtr cv_rgb;
  try {
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);//rosの型からcv::Mat型にした
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat color = cv_rgb->image;
  std::vector<cv::Mat> sources;
  sources.push_back(color);
  cv::Mat display = color.clone();
  cv::Mat drawBordering_display = color.clone(); //型取り用関数
  
  cv::Point mouse(Mouse::x(), Mouse::y());
  int event = Mouse::event();

  cv::Point roi_offset(roi_size.width / 2, roi_size.height / 2);
  // top left
  cv::Point pt1 = mouse - roi_offset;
  // bottom right
  cv::Point pt2 = mouse + roi_offset;

  cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);//矩形の描画
  cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 1);//矩形の描画

  // Perform matching  物体検出実行
  std::vector<cv::linemod::Match> matches[object_num];
  std::vector<std::string> class_ids[object_num];
  std::vector<cv::Mat> quantized_images[object_num];

  int classes_visited[object_num] = {0};
  std::set<std::string> visited[object_num];
  
  for(int k=0; k<object_num; k++){
    match_timer.start();
    detector[k]->match(sources, (float)matching_threshold, matches[k], class_ids[k], quantized_images[k]);
    match_timer.stop();
    

    for (int i = 0; (i < (int)matches[k].size()) && (classes_visited[k] < num_classes[k]); ++i) {
      // cv::linemod::Match m = matches[i];
      cv::linemod::Match m = matches[k][0];
      const std::vector<cv::linemod::Template>& templates = detector[k]->getTemplates(m.class_id, m.template_id);

      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      // pose obj3dpose;
      double quat_x = 0;
      double quat_y = 0;
      double quat_z = 0;
      double quat_w = 0;

      if (visited[k].insert(m.class_id).second) {
	++classes_visited[k];

	if (show_match_result) {
	  std::cout << "ObjectName:"<<" "<< FILENAME.at(k) <<std::endl;
	  printf("Similarity:%5.1f%%;   x:%3d;   y:%3d;\n",m.similarity,m.x,m.y);//m.x m.yは検出した物体のBboxの左上を指す
	  printf("Class:%s\n",m.class_id.c_str());
	  printf("Template:%3d\n",m.template_id);
	  // split3dPose(m, roll, pitch, yaw);
	  std::cout <<"num_mo"<< num_modalities[k] <<std::endl;
	  split3dPose(m, quat_x, quat_y, quat_z, quat_w);

	}
	
	std::cout << quat_x <<" "<< quat_y <<" "<<quat_z<<" "<<quat_w<<std::endl;

	//Visualize output on image 画像の中に検出したものを可視化する
	drawResponse(templates, num_modalities[k], display, drawBordering_display, cv::Point(m.x, m.y), detector[k]->getT(0));
	sources.clear();//vectorの中身を消去する
	sources.shrink_to_fit();//clear()だけではvectorのメモリサイズは変化しない．右の関数を使ってメモリサイズをゼロにする必要がある  
	sources.push_back(display);
        
	int similarity = (int)m.similarity; //テキスト表示のための類似度の変数
	std::string result_text = std::string(std::to_string(similarity))+"%";
	cv::putText(display, result_text, cv::Point2i(m.x, m.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);

	//center of bounding box
	cv::Point obj_center(matches[k][0].x + templates[0].width/2, matches[k][0].y + templates[0].height/2);
	//left top of bounding box
	// cv::Point img_position(matches[0].x, matches[0].y);

	/* depth */////////////////////////////////////////////////////////////
	printf("---(u,v,d)---\n");

	int bb_area = BB_AREA;
	static float minimum_depth;
	std::vector<float> depth_buf;
	cv::Point top_left(obj_center.x-(bb_area/2), obj_center.y-(bb_area/2));
	// for(int i=0; i<bb_area; i++){
	//   for(int j=0; j<bb_area; j++){
	//     float depth_area_y = top_left.y+j;
	//     float depth_area_x = top_left.x+i;
	//     float tmp_depth = (depth.at<cv::Vec3f>(depth_area_y, depth_area_x)[1]);//画像のdepthを取る
	//     cv::circle(display,cv::Point(depth_area_x,depth_area_y),3,cv::Scalar(100,100,100),5,1);
	//     if(tmp_depth < BACKGROUND_FILTER && tmp_depth > FRONT_FILTER){
	//       depth_buf.push_back(tmp_depth);
	//     }
	//   }
	//	}
	static float obj_z = 0.3;
	// float tmp_result = 0;
	// if(depth_buf.size() != 0){
	//   for(int i=0; i<depth_buf.size(); i++){
	//     tmp_result += depth_buf.at(i);
	//   }
	//   obj_z = (tmp_result/(float)depth_buf.size());
	// }
	
	// if(depth_buf.size()==0){
	// }else{
	//   obj_z = *min_element(depth_buf.begin(),depth_buf.end());
	// }
	


	///////////////////////////////////////////////////////////////////////
	std::cout << obj_center.x << ", " << obj_center.y << ", "<< obj_z <<  std::endl;
	
	
	
      	obj_pose_.header.frame_id = std::to_string(k);
	//	obj_pose_.header.frame_id = FILENAME.at(k);
	std::cout <<"obj_No: "<< obj_pose_.header.frame_id << std::endl;
	obj_pose_.pose.position.x = obj_center.x;
	obj_pose_.pose.position.y = obj_center.y;
	obj_pose_.pose.position.z = obj_z;
	obj_pose_.pose.orientation.x = quat_x;
	obj_pose_.pose.orientation.y = quat_y;
	obj_pose_.pose.orientation.z = quat_z;
	obj_pose_.pose.orientation.w = quat_w;
	obj_pose_pub_.publish(obj_pose_);      
      }
    }
      if (show_match_result && matches[k].empty())
	std::cout <<"["<< FILENAME.at(k) <<"]"<<" was not detected "<< std::endl;
      if (show_timings) {
	printf("Training: %.2fs\n", extract_timer.time());
	printf("Matching: %.2fs\n", match_timer.time());
      }
      if (show_match_result || show_timings){
	printf("------------------------------------------------------------\n");
	if( k == object_num-1){
	  printf("------------------------------------------------------------\n");
	}
      }


  }//for(k) end
  
  display = (display + drawBordering_display)/2;
  cv::imshow("color", display);


  //これいるの？→cv::FileStorage fs;
  char key = (char)cvWaitKey(10);

  switch (key) {
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
  case '[':
    // decrement threshold しきい値を下げる
    matching_threshold = std::max(matching_threshold - 1, -100);
    printf("New threshold: %d\n", matching_threshold);
    break;
  case ']':
    // increment threshold しきい値を上げる
    matching_threshold = std::min(matching_threshold + 1, +100);
    printf("New threshold: %d\n", matching_threshold);
    break;

  default:
    ;
  }
}
 


int main(int argc, char** argv) {
  ros::init(argc, argv, "linemod_abe");                  //ノードの作成
  cv::namedWindow("color", CV_WINDOW_NORMAL);            //ウィンドウに名前をつける
  cv::resizeWindow("color", WINDOW_WIDTH, WINDOW_HEIGHT);//ウィンドウのサイズを変更
  Mouse::start("color");                                 //colorウィンドウズ上でマウスの動作をさせる

  if (argc == 1) {
    /* linemod認識側ではargc==1になることはない */
  } else {
    
    for(int k=0; k<object_num ;k++){
      std::cout << k << "   " << argv[k+1] << "\n";
      FILENAME.push_back(argv[k+1]);
    }
    
    for(int k=0; k<object_num; k++){
      detector[k] = readLinemod(argv[k+1]);  //ymlファイルの名をstringで引数として渡す
      std::vector<std::string> ids = detector[k]->classIds();
      num_classes[k] = detector[k]->numClasses();
      printf("\n Loaded %s with %d classes and %d templates\n",argv[k], num_classes[k], detector[k]->numTemplates());
      if (!ids.empty()) {
	//printf("Class ids:\n");
	//std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
      }
    }
     
  rosLinemod lm;
  ros::spin();
  return 0;
  }
}

