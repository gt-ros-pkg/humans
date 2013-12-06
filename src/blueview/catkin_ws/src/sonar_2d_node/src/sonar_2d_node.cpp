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
     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
     
     ros::Rate loop_rate(10);

     
     sonar::Sonar sonar_(0,argv[1],0,20);
     sonar_.net_init("blah");
     cout << "Pings: " << sonar_.getNumPings();

     while (ros::ok()) {
          cv::Mat img;
          //sonar_.getNextSonarImage(img);
          sonar_.getSonarImage(img, -1);
          
          cv::imshow("sonar", img);
          cv::waitKey(10);
          

          ros::spinOnce();
          loop_rate.sleep();
     }


     return 0;
}
