#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>

#include <syllo_common/SylloNode.h>
#include <syllo_blueview/Sonar.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <boost/filesystem.hpp>

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

void EnableSonarLoggingCallback(const std_msgs::Bool::ConstPtr& msg)
{
     sonar.SonarLogEnable(msg->data);
}

int main(int argc, char **argv)
{              
     ros::init(argc, argv, "sonar_2d_node");
     ros::NodeHandle nh;
     
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
     ros::Subscriber min_range_sub = nh.subscribe("sonar_min_range", 1, 
                                                 MinRangeCallback);
     ros::Subscriber max_range_sub = nh.subscribe("sonar_max_range", 1, 
                                                 MaxRangeCallback);

     ros::Subscriber enable_log_sub = nh.subscribe("sonar_enable_log", 1, 
                                                   EnableSonarLoggingCallback);


     
     //Subscribe to log sonar file commands

     //Publish opencv image of sonar
     image_transport::ImageTransport it(nh);
     image_transport::Publisher image_pub;
     image_pub = it.advertise("sonar_image", 1);
          
     // cv bridge static settings:
     cv_bridge::CvImage cvi;
     cvi.header.frame_id = "image";
     cvi.encoding = "bgra8"; // sonar image is four channels

     // Image sensor message
     sensor_msgs::Image msg;

     while (ros::ok()) {          
          cv::Mat img;          
          int status = sonar.getNextSonarImage(img);
                    
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
          //cv::imshow("original", img);
          //cv::waitKey(1);
          node.spin();
     }
     node.cleanup();
     return 0;
}
