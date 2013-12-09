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
     
     typedef enum DataMode{
          image = 0,
          range
     }DataMode_t;

     Sonar();
     //Sonar(SonarMode_t mode, std::string ip_or_file, int min_range, int max_range);
     ~Sonar();
     int getNumPings();
     int getCurrentPingNum();
     void setFrameNum(int num);
     int getSonarImage(cv::Mat &image, int index);
     int getNextSonarImage(cv::Mat &image);
     int reset();
     int init();
     //int net_init(const std::string &ip);

     void set_mode(SonarMode_t mode);
     void set_data_mode(DataMode_t data_mode);
     void set_ip_addr(const std::string &ip_addr);
     void set_input_son_filename(const std::string &fn);
     void set_range(int min_range, int max_range);
     void set_color_map(const std::string &color_map);

     int SonarLogEnable(bool enable);

     int height();
     int width();

protected:
     bool initialized_;     
     std::string fn_;
     std::string ip_addr_;    
     bool logging_;

     SonarMode_t mode_;
     DataMode_t data_mode_;
     
     std::string cur_log_file_;


#if ENABLE_SONAR == 1
     BVTHead head_;
     BVTSonar son_;

     BVTMagImage img_;
     BVTColorImage cimg_;
     BVTColorMapper mapper_;
     std::string color_map_;

     // Sonar file save / logger members
     BVTSonar son_logger_;
     BVTHead out_head_;

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
