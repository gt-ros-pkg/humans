#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include <syllo_common/SylloNode.h>
#include <syllo_blueview/Sonar.h>
#include <syllo_common/Utils.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <boost/filesystem.hpp>

#include <videoray/Notes.h>

using std::cout;
using std::endl;

// Create a Sonar
Sonar sonar;     

void MinRangeCallback(const std_msgs::Float32::ConstPtr& msg)
{
     sonar.set_min_range(msg->data);
}

void MaxRangeCallback(const std_msgs::Float32::ConstPtr& msg)
{
     sonar.set_max_range(msg->data);
}

//// Open camera for recording
cv::VideoWriter record_;
bool record_initialized_ = false;
std::string video_filename_ = "";
bool logging_enabled_ = false;
cv_bridge::CvImagePtr cv_img_ptr_;
bool cv_ptr_valid_ = false;

std::string notes_filename_ = "";
std::fstream notes_file_;
std::string log_filename_ = "";
std::fstream log_file_;
int log_frame_num_ = 0;

void videoCallback(const sensor_msgs::ImageConstPtr &msg)
{
     cv_img_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");      
     cv_ptr_valid_ = true;
}

void notesCallback(const videoray::NotesConstPtr &msg)
{     
     if (!notes_file_.is_open()) {
          notes_file_.open(notes_filename_.c_str());
     }
     notes_file_ << "-------------------------------------------" << endl;
     notes_file_ << "Time: " << msg->header.stamp.sec << " " ;
     notes_file_ << msg->header.stamp.nsec << endl;
     notes_file_ << msg->notes << endl;
}

void EnableSonarLoggingCallback(const std_msgs::Bool::ConstPtr& msg)
{
     logging_enabled_ = msg->data;

     sonar.SonarLogEnable(msg->data);
     if (logging_enabled_) {
          // Generate the avi file name          
          video_filename_ = sonar.current_sonar_file() + ".avi";              
          log_filename_ = sonar.current_sonar_file() + ".txt";          
          notes_filename_ = sonar.current_sonar_file() + ".notes";
          log_frame_num_ = 0;
     } else {
          log_file_.close();
          notes_file_.close();
     }
     
     record_initialized_ = false;     
}

int main(int argc, char **argv)
{              
     ros::init(argc, argv, "sonar_2d_node");     
     ros::NodeHandle nh_;
     
     SylloNode node;
     node.init();          
     
     ///////////////////////////////////////////////////
     // Acquire params from paramserver
     ///////////////////////////////////////////////////
     // Grab minimum distance
     double min_dist;
     node.get_param("~min_dist", min_dist);

     // Grab maximim distance
     double max_dist;
     node.get_param("~max_dist", max_dist);

     // Set sonar range
     sonar.set_range(min_dist, max_dist);

     // Grab mode (image or range)
     std::string mode;
     node.get_param("~mode", mode);
     if (mode == "range") {
          sonar.set_data_mode(Sonar::range);
     } else {
          sonar.set_data_mode(Sonar::image);
     }

     // Grab color map filename
     std::string color_map;
     node.get_param("~color_map", color_map);
     sonar.set_color_map(color_map);

     // Grab sonar save directory
     std::string save_directory;
     node.get_param("~save_directory", save_directory);
     sonar.set_save_directory(save_directory);     

     notes_filename_ = save_directory + "/" + syllo::get_time_string() + ".notes";

     // Create the save_directory if it doesn't exist
     boost::filesystem::path dir(save_directory);
     if( !(boost::filesystem::exists(dir))) {
          if(boost::filesystem::create_directory(dir)) {
               std::cout << "Created new sonar save directory: " << 
                    save_directory << endl;
          } else {
               cout << "Failed to create directory: " << save_directory << endl;
          }
     }

     // Determine if a live "net" sonar will be used or if we are reading
     // from a file
     std::string net_or_file;
     node.get_param("~net_or_file", net_or_file);         

     if (net_or_file == "net") {
          // Grab suggested ip address
          std::string ip_addr;
          node.get_param("~ip_addr", ip_addr);
          sonar.set_mode(Sonar::net);
          sonar.set_ip_addr(ip_addr);
     } else {
          // Grab sonar file name
          std::string sonar_file;
          node.get_param("~sonar_file", sonar_file);
          sonar.set_mode(Sonar::sonar_file);
          sonar.set_input_son_filename(sonar_file);
     }

     // Initialize the sonar
     sonar.init();
     
     //Subscribe to range commands
     ros::Subscriber min_range_sub = nh_.subscribe("sonar_min_range", 1, 
                                                 MinRangeCallback);
     ros::Subscriber max_range_sub = nh_.subscribe("sonar_max_range", 1, 
                                                 MaxRangeCallback);

     ros::Subscriber enable_log_sub = nh_.subscribe("sonar_enable_log", 1, 
                                                   EnableSonarLoggingCallback);

     ros::Subscriber video_sub = nh_.subscribe("/videoray/camera/image_raw", 1, videoCallback);
     ros::Subscriber notes_sub = nh_.subscribe("/rqt_experiment_notes/experiment_notes", 1, notesCallback);
     //record_.open("/home/syllogismrxs/sonar_log/video.avi");         
     
     //Subscribe to log sonar file commands

     //Publish opencv image of sonar
     image_transport::ImageTransport it(nh_);
     image_transport::Publisher image_pub;
     image_pub = it.advertise("sonar_image", 1);
          
     // cv bridge static settings:
     cv_bridge::CvImage cvi;
     cvi.header.frame_id = "image";
     cvi.encoding = "bgra8"; // sonar image is four channels

     // Image sensor message
     sensor_msgs::Image msg;

     cv::Mat temp;
     sonar.getNextSonarImage(temp);

     //cv::VideoWriter record_;
     //record_.open("/home/videoray/sonar_log/video.avi", CV_FOURCC('D','I','V','X'), 10, temp.size(), true);

     ros::Time ros_time;
     
     while (ros::ok()) {          
          cv::Mat img;          
          int status = sonar.getNextSonarImage(img);
      
          // Handle video logging
          if (logging_enabled_ && cv_ptr_valid_) {
               if (!record_initialized_) {
                    //record_.open(video_filename_,CV_FOURCC('M','J','P','G'),30,cv_img_ptr_->image.size(), true);
                    record_.open(video_filename_,CV_FOURCC('D','I','V','X'),15,cv_img_ptr_->image.size(), true);
                    record_initialized_ = true;
                    log_file_.open(log_filename_.c_str(), std::ios::out);
                    if (notes_file_.is_open()) {
                         notes_file_.close();
                    }
                    notes_file_.open(notes_filename_.c_str(), std::ios::out);
               }
               record_ << cv_img_ptr_->image;

               std::ostringstream frame_num_ss, time_sec_ss, time_nsec_ss;
               frame_num_ss << log_frame_num_++;
               ros_time = ros::Time::now();
               time_sec_ss << ros_time.sec;
               time_nsec_ss << ros_time.nsec;
               
               // frame number, time (sec), time (nsec)
               log_file_ << frame_num_ss.str() << "," << time_sec_ss.str()
                         << "," << time_nsec_ss.str() << endl;
          }

	  //cap_ >> video;
	  //record_ << video;
	 
          if (status == Sonar::Success) {
               try {
                    // convert OpenCV image to ROS message
                    cvi.header.stamp = ros::Time::now();
                    cvi.image = img;
                    cvi.toImageMsg(msg);
               
                    // Publish the image
                    image_pub.publish(msg);
	  
               } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return -1;
               }
          }

	  
          //cv::imshow("sonar", img);
	  //cv::imshow("video", video);
          //cv::waitKey(1);
          node.spin();
     }
     node.cleanup();
     return 0;
}
