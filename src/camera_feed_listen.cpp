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

//images for various stages of processing
Mat img, img_grey, img_thresh, img_dil, img_inv, img_blobs, img_flip, img_video_blobs, img_final;

class ImageConverter
{
  /*ROS handles*/
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher beacon_flag_pub = nh_.advertise<std_msgs::Int8>("beacon_flag",10);
  ros::Publisher beacon_heading_pub = nh_.advertise<std_msgs::Int8>("beacon_heading",10);
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
  }

  ~ImageConverter()
  {

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

	      img_flip =cv_ptr->image;
        // select a region of interest
        cv::Mat part_ignore = img_flip(cv::Rect(0, 0, 640, 100));
        part_ignore.setTo(cv::Scalar(0, 0, 0));
        //bottom ignore
        cv::Mat part_ignore_2 = img_flip(cv::Rect(0, 440, 640, 40));
        part_ignore_2.setTo(cv::Scalar(0, 0, 0));
        //left ignore
        cv::Mat part_ignore_3 = img_flip(cv::Rect(0, 0, 25, 480));
        part_ignore_3.setTo(cv::Scalar(0, 0, 0));
        //right ignore
        cv::Mat part_ignore_4 = img_flip(cv::Rect(615, 0, 25, 480));
        part_ignore_4.setTo(cv::Scalar(0, 0, 0));

        //convert to grey
        cvtColor( img_flip, img_grey, CV_BGR2GRAY);

        // Threshold image
        threshold( img_grey, img_thresh, 226, max_BINARY_value, 3);

        //dilation element size and shape. Used in dilation
        Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );

        //Dilate edges
        dilate(img_thresh, img_dil,element);
        //invert image
        bitwise_not(img_dil, img_inv);

        // Threshold image
        threshold( img_inv, img_final, 230, max_BINARY_value, 0);

        //sum each elememt, used for saturation calculation
        double s = cv::sum(img_final)[0] / max_BINARY_value;
        s = s/480/640;
        cout<<"Count is : "<<s<<endl;

        // Change thresholds
        params.minThreshold = 232;
        params.maxThreshold = 255;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 1000;
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
        params.maxInertiaRatio = 0.8;

        // Storage for blobs
        vector<KeyPoint> keypoints;

        // Set up detector with params
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

        // Detect blobs
        detector->detect( img_final, keypoints);
        size_t blob_count=keypoints.size();

        /*Iterate over keypoints array and store array location of largest blob
        Used to find only largest blob (beacon) if reflections are found*/
       for (int i=0;i<keypoints.size();i++){

             if (keypoints[i].size>temp_size){
               temp_size=keypoints[i].size;
               temp = i;
             }

          //set value of largest blob and its size to a variable
          blob_x = keypoints[temp].pt.x;
          blob_x_size = keypoints[temp].size;
      }

      /*Determine the angle to the blob*/
      angle_sum = -1*(blob_x-320);
      angle = atan(angle_sum/240) * 180/PI;
      angle = angle + 0.5;
      int angle_final = (int)angle;

      //draw blob circles on the image for user viewing
      drawKeypoints( img_final, keypoints, img_blobs, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      drawKeypoints( img_flip, keypoints, cv_ptr->image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    //define message topic flags
    std_msgs::Int8 msg_flag;
    std_msgs::Int8 msg_head;

    //if saturation is greater than 25% flag beacon found
    if(s <= 0.75){
        msg_flag.data = 2;
    }else{

            if (blob_count == 0){
              msg_flag.data = 0;

            }else{
              //if blob is found publish found flag and angle to it
              msg_flag.data = 1;
              msg_head.data = angle_final;
              //publish msg_head topic
              beacon_heading_pub.publish(msg_head);
  }
}
    //Publish msg_flag topic
    beacon_flag_pub.publish(msg_flag);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
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
