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
     ros::init(argc, argv, "talker");
     ros::NodeHandle n;
     
     SylloNode node_;
     node_.init();

     // Determine if a live "net" sonar will be used or if we are reading
     // from a file
     std::string net_or_file;
     node_.get_param("~net_or_file", net_or_file);
     
     if (net_or_file == "net") {
          // Grab suggested ip address
          std::string ip_addr;
          node_.get_param("~ip_addr", ip_addr);
     } else {
          // Grab sonar file name
          std::string sonar_file;
          node_.get_param("~sonar_file", sonar_file);
     }

     // Grab minimum distance
     double min_dist;
     node_.get_param("~min_dist", min_dist);

     // Grab maximim distance
     double max_dist;
     node_.get_param("~max_dist", max_dist);

     // Grab mode (image or range)
     std::string mode;
     node_.get_param("~mode", mode);
     
     //std::string 
     

     //Sonar sonar_(,argv[1],0,20);
          
     //sonar::Sonar sonar_(0,argv[1],0,20);
     //sonar_.net_init("blah");
     //cout << "Pings: " << sonar_.getNumPings();
     //
     //while (ros::ok()) {
     //     cv::Mat img;
     //     //sonar_.getNextSonarImage(img);
     //     sonar_.getSonarImage(img, -1);
     //     
     //     cv::imshow("sonar", img);
     //     cv::waitKey(10);
     //     
     //
     //     ros::spinOnce();
     //     loop_rate.sleep();
     //}


     return 0;
}
