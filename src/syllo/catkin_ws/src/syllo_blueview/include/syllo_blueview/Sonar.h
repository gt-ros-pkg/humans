#ifndef _SONAR_H_
#define _SONAR_H_

#define ENABLE_SONAR 1

#include <cv.h>

#if ENABLE_SONAR == 1
#include <bvt_sdk.h>
#endif

class Sonar {
public:
     typedef enum SonarMode{
          net = 0,
          sonar_file
     }SonarMode_t;

     Sonar();
     Sonar(SonarMode_t mode, std::string ip_or_file, int min_range, int max_range);
     ~Sonar();
     int getNumPings();
     int getCurrentPingNum();
     void setFrameNum(int num);
     int getSonarImage(cv::Mat &image, int index);
     int getNextSonarImage(cv::Mat &image);
     int reset();
     int init();
     int net_init(const std::string &ip);
     void setRange(int min_range, int max_range);
     void setSonarFile(std::string fn);

     int height();
     int width();

protected:
     std::string fn_;
     std::string ip_addr_;
     

     SonarMode_t mode_;

#if ENABLE_SONAR == 1
     BVTHead head_;
     BVTSonar son_;

     BVTMagImage img_;
     BVTColorImage cimg_;
     BVTColorMapper mapper_;
#endif

     int heads_;
     int pings_;

     int cur_ping_;

     int min_range_;
     int max_range_;
	  
     int height_;
     int width_;	 
};

#endif
