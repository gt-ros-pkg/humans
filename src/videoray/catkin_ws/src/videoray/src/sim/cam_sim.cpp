#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "videoray/Throttle.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Image.h"

#include <iostream>
#include <sstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using std::cout;
using std::endl;

#define PI (3.14159265359)

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW1[] = "Original";
static const char WINDOW2[] = "Lens";

void pincushion(cv::Mat &in, cv::Mat &out)
{
     // accept only char type matrices
     CV_Assert(in.depth() != sizeof(uchar));
     
     out = cv::Mat::zeros(in.size(), in.type());

     int channels = in.channels();
     int nRows = in.rows;
     int nCols = in.cols * channels;
     if (in.isContinuous())
     {
          nCols *= nRows;
          nRows = 1;
     }

     cout << "Image" << endl;
     cout << "Channels: " << channels << endl;
     cout << "Rows: " << nRows << endl;
     cout << "Cols: " << nCols << endl;
     cout << "IsContinuous: " << in.isContinuous() << endl;


     int i,j;
     uchar* pIn;
     uchar *pOut;
     for( i = 0; i < nRows; ++i)
     {
          pIn = in.ptr<uchar>(i);
          pOut = out.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j)
          {
               pOut[j] = pIn[j];
          }
     }

}

void cb_img(const sensor_msgs::ImageConstPtr& msg)
{
     cv_bridge::CvImagePtr cv_ptr;
     try
     {
          cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
     }
     catch (cv_bridge::Exception& e)
     {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
     }

     cv::imshow(WINDOW1, cv_ptr->image);
     cv::Mat out;
     pincushion(cv_ptr->image, out);
     cv::imshow(WINDOW2, out);
     cv::waitKey(3);
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "cam_sim");     
     ros::NodeHandle n;
     
     ros::Subscriber sub_img = n.subscribe("/videoray/videocamera/image", 1, cb_img);
     ros::Publisher pose_pub = n.advertise<sensor_msgs::Image>("video_sim",1);

     //image_transport::ImageTransport it_;
     //image_transport::Subscriber image_sub_;
     //image_transport::Publisher image_pub_;


     double rate = 10;
     ros::Rate loop_rate(rate);

     ros::Time begin = ros::Time::now();
     ros::Time curr_time = begin;
     ros::Time prev_time = begin;
     ros::Duration dt = curr_time - prev_time;
     
     while (ros::ok())
     {
          curr_time = ros::Time::now();
          dt = curr_time - prev_time;
          prev_time = curr_time;

          ros::spinOnce();
          loop_rate.sleep();
     }
     return 0;
}
