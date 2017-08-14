#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include <std_msgs/Int8.h>
#include <math.h>       /* atan */
#include <stdio.h>

#define PI 3.14159265

#define CAMERA__WIDTH 480
#define CAMERA__HEIGHT 640

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_2 = "Camera Feed";
using namespace cv;
using namespace std;

/// Global variables
double const max_BINARY_value = 255;
int dilation_elem = 0;
int dilation_size = 10;
int dilation_type = 2;
int temp_size = 0;
int temp = 0;

double angle_sum;
double angle;
double  blob_x = 0;
double blob_x_size =0;
int angle_final;

vector<Mat> channels;

Mat img, img_grey, img_thresh, img_dil, img_inv, img_blobs, img_flip, img_video_blobs, img_final, img_hsv;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //ros::Publisher beacon_flag_pub = nh_.advertise<std_msgs::String>("beacon_flag", 1000);
  ros::Publisher beacon_flag_pub = nh_.advertise<std_msgs::Int8>("beacon_flag",1000);
  ros::Publisher beacon_heading_pub = nh_.advertise<std_msgs::Int8>("beacon_heading",1000);
  int count = 0;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/raspicam_node/image_raw", 1,
      &ImageConverter::imageCb, this);
    cout << "FLAG";
    image_pub_ = it_.advertise("/ja_robbie_cam/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW_2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW_2);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

        // Setup SimpleBlobDetector parameters.
        SimpleBlobDetector::Params params;

        flip(cv_ptr->image, img_flip,0);

        // select a region of interest
        cv::Mat part_ignore = img_flip(cv::Rect(0, 0, 640, 130));
        part_ignore.setTo(cv::Scalar(0, 0, 0));
        //bottom ignore
        cv::Mat part_ignore_2 = img_flip(cv::Rect(0, 440, 640, 40));
        part_ignore_2.setTo(cv::Scalar(0, 0, 0));
        //left ignore
        cv::Mat part_ignore_3 = img_flip(cv::Rect(0, 0, 5, 480));
        part_ignore_3.setTo(cv::Scalar(0, 0, 0));
        //right ignore
        cv::Mat part_ignore_4 = img_flip(cv::Rect(635, 0, 5, 480));
        part_ignore_4.setTo(cv::Scalar(0, 0, 0));

        //threshold( img_flip, img_flip, 232, max_BINARY_value, 1);

        cvtColor( img_flip, img_hsv, CV_BGR2HSV);
      /*  split(img_hsv, channels);
        double s = cv::sum(img_hsv)[2];
        cout<<"COunt is : "<<s<<endl;
*/
        //convert to grey
        cvtColor( img_flip, img_grey, CV_BGR2GRAY);

        // Threshold image
        threshold( img_grey, img_thresh, 232, max_BINARY_value, 3);
      //    cv::sum(img_thresh);


        Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );

        //Dilate edges
        dilate(img_thresh, img_dil,element);
        //invert image
        bitwise_not(img_dil, img_inv);

        // Threshold image
        threshold( img_inv, img_final, 232, max_BINARY_value, 0);

        //split(img_hsv, channels);
        double s = cv::sum(img_final)[0] / max_BINARY_value;
        s = s/480/640;
        cout<<"Count is : "<<s<<endl;

        //cout<< img_final.at(Point(0,0))<<endl;

        //cvtColor( img_final, img_hsv, CV_BGR2HSV);
      //  split(img_hsv, channels);
        //double s = cv::sum(img_hsv)[2];


        //cout<<"COunt is : "<<s<<endl;



        // Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 200;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 2000;
        params.maxArea = 100000;

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0;

        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.1;

        // Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = 0.01;

        // Storage for blobs
        vector<KeyPoint> keypoints;

        // Set up detector with params
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);


        // Detect blobs
        detector->detect( img_final, keypoints);
        size_t blob_count=keypoints.size();
        cout<<"total no of circles detected are: "<<blob_count<<endl;

        for (int i=0;i<keypoints.size();i++){
          // cout<<keypoints[i].pt<<"\n";
          // cout<<keypoints[i].size<<"\n";
         //  keypoints[i].pt.x
       }

       for (int i=0;i<keypoints.size();i++){

             if (keypoints[i].size>temp_size){

               temp_size=keypoints[i].size;
               temp = i;

             }

          blob_x = keypoints[temp].pt.x;
          blob_x_size = keypoints[temp].size;
         // cout<<keypoints[i].size<<"\n";
        //  keypoints[i].pt.x
      }
      cout<<"Largest blob is at x = "<<blob_x<<"\n";
      cout<<"Largest blob size is = "<<blob_x_size<<"\n";

      angle_sum = -1*(blob_x-320);
      angle = atan(angle_sum/240) * 180/PI;
      angle = angle + 0.5;
      int angle_final = (int)angle;

      //circle( img_flip, Point( blob_x, 240 ), 32.0, Scalar( 0, 255, 0 ), 1, 8 );
    //  arrowedLine(img_flip, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0, double tipLength=0.1)

        //  Mat im_with_keypoints;
        drawKeypoints( img_final, keypoints, img_blobs, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( img_flip, keypoints, img_video_blobs, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      //  drawKeypoints( img_flip, keypoints[temp].pt, img_video_blobs, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img_blobs);
    cv::imshow(OPENCV_WINDOW_2, img_video_blobs);
    cv::waitKey(3);

    std_msgs::Int8 msg_flag;
    std_msgs::Int8 msg_head;
  //  msg_flag.data = 1;
  //  msg_head.data = 1;

    if(s <= 0.75){
        msg_flag.data = 2;
    }else{

            if (blob_count == 0){
              msg_flag.data = 0;

            }else{
              msg_flag.data = 1;
            //  msg_head.data = 500;
            msg_head.data = angle_final;
            beacon_heading_pub.publish(msg_head);
  }
}

  //  ROS_INFO("%d", msg_flag.data);
    beacon_flag_pub.publish(msg_flag);

    //ROS_INFO("%lf", msg_head.data);


    // Output modified video stream
   // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{

  cout << "Start";
  ros::init(argc, argv, "ja_robbie_cam");

  ImageConverter ic;

  ros::spin();
  return 0;

}
