#include "object_detection.h"
#include "camera_tracking.h"
#include "robot_move.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv4/opencv2/tracking/tracker.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/face.hpp>
#include <math.h>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <fstream>
#include <sstream>
#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
// lib for face recognize
#include "src/config.h"
#include "src/anchor_creator.h"
#include "src/utils.h"
#include "benchmark.h"
// lib for tracker
#include "src/kcftracker.hpp"
#include <dirent.h>
using namespace std;
using namespace cv;
using namespace cv::face;
using namespace cv::xfeatures2d;
// move
#define maxdistance 250
#define mindistance 180
#define MaxRight 100
#define MaxLeft 90
#define Speed 50
const int right_pwm_pin = 33; //pwm
const int right_output_pin1 = 29; //
const int right_output_pin2 = 31; //

const int left_pwm_pin = 32; //pwm
const int left_output_pin1 = 38; // 
const int left_output_pin2 = 40; // pwm
// NEW

//***************************funtion  face detect and recognize****************************//
static int init_retinaface(ncnn::Net* retinaface, const int target_size)
{
    int ret = 0;
    // gpu
    
    ncnn::create_gpu_instance();
    retinaface->opt.use_vulkan_compute = 1;
    retinaface->opt.num_threads = 8;
    retinaface->opt.use_winograd_convolution = true;
    retinaface->opt.use_sgemm_convolution = true;

    const char* model_param = "../models/retinaface.param";
    const char* model_model = "../models/retinaface.bin";
    
    ret = retinaface->load_param(model_param);
    if(ret)
    {
        return ret;
    }
    ret = retinaface->load_model(model_model);
    if(ret)
    {
        return ret;
    }

    return 0;
}
static void deinit(ncnn::Net* retinaface,ncnn::Net* mbv2facenet){ 
    retinaface->clear();
    ncnn::destroy_gpu_instance();
    mbv2facenet->clear();
}
static int init_mbv2facenet(ncnn::Net* mbv2facenet, const int target_size)
{
    int ret = 0;
    // use gpu vulkan
    ncnn::create_gpu_instance();
    mbv2facenet->opt.use_vulkan_compute = 1;
    
    mbv2facenet->opt.num_threads = 8;
    mbv2facenet->opt.use_sgemm_convolution = 1;
    mbv2facenet->opt.use_winograd_convolution = 1;

    const char* model_param = "../models/mbv2facenet.param";
    const char* model_bin = "../models/mbv2facenet.bin";

    ret = mbv2facenet->load_param(model_param);
    if(ret)
    {
      return ret;
    }

    ret = mbv2facenet->load_model(model_bin);
    if(ret)
    {
        return ret;
    }
    return 0;
}

std::vector<cv::Rect> detect_retinaface(ncnn::Net* retinaface, cv::Mat img, const int target_size, std::vector<cv::Mat>& face_det)
{
    int img_w = img.cols;
    int img_h = img.rows;
    float tempo1= img_w*1.0 /target_size;

    float tempo2= img_h*1.0 /target_size;

    cv::Mat img1;
    std::vector<cv::Rect> bor;
    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1, 1, 1};

    ncnn::Mat input = ncnn::Mat::from_pixels_resize(img.data, ncnn::Mat::PIXEL_BGR2RGB,img_w, img_h, target_size, target_size);
    input.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = retinaface->create_extractor();
    ex.set_vulkan_compute(true);
    std::vector<AnchorCreator> ac(_feat_stride_fpn.size());
    for(size_t i = 0; i < _feat_stride_fpn.size(); i++)
    {
        int stride = _feat_stride_fpn[i];
        ac[i].init(stride, anchor_config[stride], false);
    }
    
    ex.input("data", input);

    std::vector<Anchor> proposals;

    for(size_t i = 0; i < _feat_stride_fpn.size(); i++)
    {
        ncnn::Mat cls;
        ncnn::Mat reg;
        ncnn::Mat pts;
        char cls_name[100];
        char reg_name[100];
        char pts_name[100];
        sprintf(cls_name, "face_rpn_cls_prob_reshape_stride%d", _feat_stride_fpn[i]);
        sprintf(reg_name, "face_rpn_bbox_pred_stride%d", _feat_stride_fpn[i]);
        sprintf(pts_name, "face_rpn_landmark_pred_stride%d", _feat_stride_fpn[i]);

        ex.extract(reg_name,reg);
        ex.extract(pts_name,pts);

        ac[i].FilterAnchor(cls, reg, pts, proposals);
    }

    std::vector<Anchor> finalres;
    box_nms_cpu(proposals, nms_threshold, finalres, target_size);
    cv::resize(img, img, cv::Size(target_size, target_size));
    for(size_t i = 0; i < finalres.size(); ++i)
    {
        finalres[i].print();//in thong so khuon mat xy-width-height-threadshold 300x300
        cv::Mat face = img(cv::Range((int)finalres[i].finalbox.y, (int)finalres[i].finalbox.height),cv::Range((int)finalres[i].finalbox.x, (int)finalres[i].finalbox.width)).clone();
        face_det.push_back(face);
        
        /*cv::rectangle(img, cv::Point((int)finalres[i].finalbox.x, (int)finalres[i].finalbox.y), cv::Point((int)finalres[i].finalbox.width, (int)finalres[i].finalbox.height),cv::Scalar(255,255,0), 2, 8, 0);*/
        float x=(float)finalres[i].finalbox.x*tempo1;
        float y= (float)finalres[i].finalbox.y*tempo2;
        float width =(float)finalres[i].finalbox.width*tempo1-x;
        float heigh =(float)finalres[i].finalbox.height*tempo2-y;
        bor.push_back(cv::Rect((int)x,(int)y,(int)width,(int)heigh));
    }
    return bor;
}

void run_mbv2facenet(ncnn::Net* mbv2facenet, std::vector<cv::Mat>& img, int target_size, std::vector<std::vector<float>>& res)
{
    for(size_t i = 0; i < img.size(); ++i)
    {
        ncnn::Extractor ex = mbv2facenet->create_extractor();
        ex.set_vulkan_compute(true);
        ncnn::Mat input = ncnn::Mat::from_pixels_resize(img[i].data, ncnn::Mat::PIXEL_BGR2RGB, img[i].cols, img[i].rows, target_size, target_size);
        ex.input("data", input);
        
        ncnn::Mat feat;

        ex.extract("fc1", feat);

        std::vector<float> tmp;
        for(int i = 0; i < feat.w; ++i)
        {
            tmp.push_back(feat.channel(0)[i]);
        }
        res.push_back(tmp);
    }
}   

//****************************************end func **************************************
int pan = 90;
int serial_port;

cv::Rect2d r;

cv::Point CalCenterObject(cv::Rect r)
{
  cv::Point point;
  point.x = r.x + r.width/2;
  point.y = r.y + r.height/2;
  return point;
}


void frameWriter(Mat writeFrame, VideoWriter *orgVideo, mutex *mutexVid)
{
    (mutexVid->lock());
    (orgVideo->write(writeFrame));
    (mutexVid->unlock());
}

int main( int argc, char** argv )
{

  GPIO::setwarnings(false);
	GPIO::setmode(GPIO::BOARD);

	// set pin as an output pin with optional initial state of HIGH
  GPIO::setup(right_pwm_pin, GPIO::OUT, GPIO::HIGH);	
  GPIO::setup(right_output_pin1, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(right_output_pin2, GPIO::OUT, GPIO::LOW);
  GPIO::PWM p_right(right_pwm_pin, 50);

  GPIO::setup(left_pwm_pin, GPIO::OUT, GPIO::HIGH);	
	GPIO::setup(left_output_pin1, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(left_output_pin2, GPIO::OUT, GPIO::LOW);
  GPIO::PWM p_left(left_pwm_pin, 50);
  //-----------------------------DETECT FACE-------------------------------
  //face recognizer 

  int target_size = 300;
  int facenet_size = 112;
  ncnn::Net retinaface;
  ncnn::Net mbv2facenet;  
  bool RUNFACE_DETECTION=true;
  int ret = 0;

  cv::Mat img1 = cv::imread("../Target_Face/Tung.jpg", 1);
  ret = init_retinaface(&retinaface, target_size);
  if(ret)
  {
      cout<<"loi load model init_retinaface() "<<endl;
      return -1;
  }

  ret = init_mbv2facenet(&mbv2facenet, facenet_size);
  if(ret)
  {
      cout<<"loi load model nit_mbv2facenet() "<<endl;
      return -1;
  }
  std::vector<cv::Mat> face_det1;
  std::vector<std::vector<float>> feature_face1;
  auto box1 = detect_retinaface(&retinaface, img1, target_size, face_det1); //pic1 dect
  run_mbv2facenet(&mbv2facenet, face_det1, facenet_size, feature_face1);
  //-----------------------------DETECT FACE-------------------------------END
  cv::VideoCapture cap("/dev/video1");
  cv::Mat frame;

  ObjectDetection det("../../pedestrian_detection/");
  
  auto m_StartTime = std::chrono::system_clock::now();

  bool isInitTracking = false;

  //config tracker
  bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool LAB = false;
  // Create KCFTracker object
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

  cv::Point center_face_detect;
  cv::Point center_person_detect;
  cv::Rect BoudBox_Target;
  cv::Rect BoudBox_Tracking;
  cv::Mat cropTarget_1;
  float time_test = 0;
  int nFrame = 0;
  int nFrameTest = 0;
  int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
  VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));
  mutex modVideoMutex;
  while(true)
  {
    nFrameTest++;
    cap >> frame;
    double fps = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_StartTime).count();
    m_StartTime = std::chrono::system_clock::now();
    cv::putText(frame, to_string(static_cast<int>(1000/fps)) + " FPS", cv::Point(10, 30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 1, false);
  
    //-----------------------------------Detect face -----------------------------
    
    if (RUNFACE_DETECTION == true) {
      cv::Mat img2 = frame.clone();
      std::vector<cv::Mat> face_det;
      std::vector<std::vector<float>> feature_face;
      
      auto box2=detect_retinaface(&retinaface, img2, target_size, face_det);//pic2 dect
      if(face_det.size() >= 1) 
      {
        run_mbv2facenet(&mbv2facenet, face_det, facenet_size, feature_face);
        for(int i=0; i<int(feature_face.size());i++){
          float sim = calc_similarity_with_cos(feature_face1[0], feature_face[i]);
          cv::rectangle(frame,box2[i],cv::Scalar(255,255,0), 2, 8, 0);
          string text;
          if(sim >= 0.3)
          {
            text ="object";
            cv::putText(frame, text, cv::Point(box2[i].x - 10, box2[i].y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            // center_face_detect = CalCenterObject(box2[i].x, box2[i].y, box2[i].width, box2[i].height);
            center_face_detect = CalCenterObject(box2[i]);
            RUNFACE_DETECTION = false;
            break;
          }
          else
          {
            text ="unknow";
            cv::putText(frame, text, cv::Point(box2[i].x - 10, box2[i].y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
          }
            cout<<"Same ratio of Face: "<< sim<<endl;
        }
      }

		}
    else if (RUNFACE_DETECTION == false && isInitTracking == false){
      std::cout << "Run person detection ...................." <<std::endl;
      auto recs = det.detectObject(frame);
      if (recs.empty()){
        // std:: cout<<"no person to detect"<<std::endl;
      }
      else{
      float dis_Face_Person = 1000;
      
      for(auto rec:recs)
      {
        // cv::rectangle(frame, rec, cv::Scalar(0, 255, 0), 2);

        center_person_detect = CalCenterObject(rec);
        if (abs(center_person_detect.x - center_face_detect.x) < dis_Face_Person){
          dis_Face_Person = abs(center_person_detect.x - center_face_detect.x);
          BoudBox_Target = rec;
        }
      }
        if(!BoudBox_Target.empty())
          {
            tracker.init(BoudBox_Target, frame);
          }
        if(0 <= BoudBox_Target.x
              && 0 <= BoudBox_Target.width
              && BoudBox_Target.x + BoudBox_Target.width <= frame.cols
              && 0 <= BoudBox_Target.y
              && 0 <= BoudBox_Target.height
              && BoudBox_Target.y + BoudBox_Target.height <= frame.rows)
        cropTarget_1 = frame(BoudBox_Target);

        isInitTracking = true;
    }	
    
    }
    if(isInitTracking == true && nFrame == 5)
    {
      nFrame = 0;
      bool isUpDate = true;
      auto recs = det.detectObject(frame);
      cv::Rect temp;
      if (recs.empty()){
      }
      else{
        float dis_person = 1000;
        float cen_box_ex = (CalCenterObject(BoudBox_Target)).x;
        int count = 0;
        for(auto rec:recs){
          if (0 <= rec.x
              && 0 <= rec.width
              && rec.x + rec.width <= frame.cols
              && 0 <= rec.y
              && 0 <= rec.height
              && rec.y + rec.height <= frame.rows && !rec.empty()){

          count++;      
          float dis_cal = abs((CalCenterObject(rec)).x - cen_box_ex);
          if(count != 1){
          if(((rec.x >= temp.x) && (rec.x <= temp.x + temp.width)) || ((rec.x+rec.width >= temp.x) && (rec.x+rec.width <= temp.x + temp.width)))
            {
              cout<<"false";
              isUpDate = false;
              break;
            }
          } 
          if(dis_cal < dis_person){
            dis_person = dis_cal;
            temp = rec;
          }
        }
        }
          if(isUpDate == true && count != 0){
          // std::cout << "Update done ...................." <<std::endl;
          tracker.init(temp, frame);
          // std::cout << "Update done ...................." <<std::endl;
          }
          BoudBox_Tracking = tracker.update(frame);
          cv::rectangle(frame, BoudBox_Tracking, cv::Scalar(0, 255, 0), 2, 1);
          string a = std::to_string(count);
          cv::putText(frame, a, cv::Point(BoudBox_Target.x -10, BoudBox_Target.y -10),
          cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
      }
    }

    else if (isInitTracking == true){
      nFrame = nFrame + 1;
      BoudBox_Tracking = tracker.update(frame);
      cv::rectangle(frame, BoudBox_Tracking, cv::Scalar(0, 0, 255), 2, 1);
      int distance = Distance(BoudBox_Tracking.width, BoudBox_Tracking.height, frame_width, frame_height);
      string distancee = std::to_string(distance);
      cout<<"distacne"<<distancee<<endl;
      cv::putText(frame, distancee, cv::Point(BoudBox_Target.x +30, BoudBox_Target.y +30),
      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);


      int objX =  BoudBox_Tracking.x + BoudBox_Tracking.width/2; 
      int errorPan = objX - frame_width/2;
      if(abs(errorPan) > 100)
      {
        if(errorPan > 0) //Taget in right of frame 
          {
            p_right.stop();
            p_left.start(MaxLeft-60);
            GPIO::output(left_output_pin1, GPIO::HIGH);
            GPIO::output(left_output_pin2, GPIO::LOW);
          }
        else       //Taget in left of frame
          {
            p_left.stop();
            p_right.start(MaxRight-60);
            GPIO::output(right_output_pin1, GPIO::HIGH);
            GPIO::output(right_output_pin2, GPIO::LOW);
          }
      }
      else {
        if(distance > maxdistance)
        {
          GPIO::output(right_output_pin1, GPIO::HIGH);
          GPIO::output(right_output_pin2, GPIO::LOW);

          GPIO::output(left_output_pin1, GPIO::HIGH);
          GPIO::output(left_output_pin2, GPIO::LOW);
           cout<<"tien"<<endl;
          p_right.start(MaxRight - Speed);
          p_left.start(MaxLeft - Speed);
        }
      else if(mindistance < distance && distance < maxdistance)
      {
       cout<<"stop"<<endl;
       p_left.stop();
       p_right.stop();

      }
      else
      {
         cout<<"Lui"<<endl;
        GPIO::output(right_output_pin1, GPIO::LOW);
        GPIO::output(right_output_pin2, GPIO::HIGH);

        GPIO::output(left_output_pin1, GPIO::LOW);
        GPIO::output(left_output_pin2, GPIO::HIGH);

        p_right.start(MaxRight - Speed);
        p_left.start(MaxLeft - Speed);
      }
      }
    }
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now()-m_StartTime);
    time_test += time_used.count();
    cv::imshow("detection", frame);
    cv::Mat temp = frame.clone();
    thread writeModThread(frameWriter, temp, &video, &modVideoMutex);
    writeModThread.detach();

    if(cv::waitKey(1) == 27)
    {
        break;
    }
    
  }
  cout <<"Time average: "<<time_test/nFrameTest<<endl;
	p_right.stop();
	p_left.stop();
	GPIO::cleanup(29);
	GPIO::cleanup(31);
	GPIO::cleanup(32);
	GPIO::cleanup(33);
	GPIO::cleanup(38);  
	GPIO::cleanup(40);

  cap.release();
  //video.release();
  cv::destroyAllWindows();
  deinit(&retinaface,&mbv2facenet);
  return 0;
}
