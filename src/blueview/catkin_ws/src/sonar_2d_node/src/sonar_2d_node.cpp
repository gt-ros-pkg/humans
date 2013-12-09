#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>

#include <syllo_common/SylloNode.h>
#include <syllo_blueview/Sonar.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;

int main(int argc, char **argv)
{              
     ros::init(argc, argv, "sonar_2d_node");
     ros::NodeHandle n;
     
     SylloNode node;
     node.init();     

     // Create a Sonar
     Sonar sonar;          
          
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

     while (ros::ok()) {
          cv::Mat img;          
          int status = sonar.getNextSonarImage(img);
                    
          cv::imshow("sonar", img);
          cv::waitKey(1);
     
          node.spin();
     }
     node.cleanup();
     return 0;
}
