#include <iostream>
#include <stdio.h>
#include <fstream>

#include <syllo_blueview/Sonar.h>
#include <syllo_common/Utils.h>

#if ENABLE_SONAR == 1
#include <bvt_sdk.h>
#endif

using std::cout;
using std::endl;

Sonar::Sonar()
     : initialized_(false), fn_(""), ip_addr_(""), logging_(false), 
       mode_(Sonar::net), data_mode_(Sonar::image), min_range_(0), 
       max_range_(40), color_map_(""), save_directory_("./")
{
}

//Sonar::Sonar(SonarMode_t mode, std::string ip_or_file, int min_range, int max_range)
//     : initialized_(false), logging_(false), data_mode_(Sonar::image)
//{
//     mode_ = mode;
//
//     if (mode_ == Sonar::net) {
//          ip_addr_ = ip_or_file;
//          fn_ = "";
//     } else {
//          fn_ = ip_or_file;
//          ip_addr_ = "";
//     }
//         
//     min_range_ = min_range;
//     max_range_ = max_range; 
//}

Sonar::~Sonar()
{
#if ENABLE_SONAR == 1
     // TODO: Fix: Causing seg faults after just init, but no grab
     //BVTColorImage_Destroy(cimg_);
     //BVTMagImage_Destroy(img_);
     //BVTColorMapper_Destroy(mapper_);
     //BVTSonar_Destroy(son_);
#endif
}
     
//int Sonar::net_init(const std::string &ip)
//{
//     //Create the discovery agent
//     BVTSonarDiscoveryAgent agent = BVTSonarDiscoveryAgent_Create();
//     if( agent == NULL )
//     {
//          printf("BVTSonarDiscoverAgent_Create: failed\n");
//          return 1;
//     }
//          
//     // Kick off the discovery process
//     int ret;
//     ret = BVTSonarDiscoveryAgent_Start(agent);
//          
//     //Let the discovery process run for a short while (5 secs)
//     cout << "Searching for sonar" << endl;
//     sleep(2);
//          
//     // See what we found
//     int numSonars = 0;
//     numSonars = BVTSonarDiscoveryAgent_GetSonarCount(agent);
//          
//     char SonarIPAddress[20];
//          
//     for(int i = 0; i < numSonars; i++)
//     {
//          ret = BVTSonarDiscoveryAgent_GetSonarInfo(agent, i, &SonarIPAddress[0], 20);
//          printf("Found Sonar: %d, IP address: %s\n", i, SonarIPAddress);
//     }
//          
//     if(numSonars == 0)
//     {
//          printf("No Sonars Found\n");
//          return(1);
//     }
//          
//     cur_ping_ = 0;
//
//     son_ = BVTSonar_Create();
//     if (son_ == NULL ) {
//          printf("BVTSonar_Create: failed\n");
//          //return 1;
//     }
//
//     // Open the sonar
//     ret = BVTSonar_Open(son_, "NET", "192.168.1.45");
//     if( ret != 0 )
//     {
//          printf("BVTSonar_Open: ret=%d\n", ret);
//          //return 1;
//     }
//
//     // Make sure we have the right number of heads_
//     heads_ = -1;
//     heads_ = BVTSonar_GetHeadCount(son_);
//     printf("BVTSonar_GetHeadCount: %d\n", heads_);
//	  
//     // Get the first head
//     head_ = NULL;
//     ret = BVTSonar_GetHead(son_, 0, &head_);
//     if (ret != 0 ) {
//
//          ret = BVTSonar_GetHead(son_, 1, &head_);
//          if (ret != 0) {
//               printf( "BVTSonar_GetHead: ret=%d\n", ret) ;
//               //return 1;
//          }
//          //return 1;
//     }
//	
//     // Set the range window to be 10m to 40m
//     BVTHead_SetRange(head_, min_range_, max_range_);
//
//     // Build a color mapper
//     mapper_ = BVTColorMapper_Create();
//     if (mapper_ == NULL) {
//          printf("BVTColorMapper_Create: failed\n");
//          //return 1;
//     }
//
//     // Load the bone colormap
//     ret = BVTColorMapper_Load(mapper_, "../sonar-processing/bvtsdk/colormaps/jet.cmap");
//     if(ret != 0) {
//          printf("BVTColorMapper_Load: ret=%d\n", ret);
//          //return 1;
//     }
//}

int Sonar::init()
{
#if ENABLE_SONAR == 1
     logging_ = false;
     cur_ping_ = 0;

     son_ = BVTSonar_Create();
     if (son_ == NULL ) {
          printf("BVTSonar_Create: failed\n");
          return -1;
     }

     int ret;
     if (mode_ == Sonar::net) {
          /////////////////////////////////////////
          // Reading from physical sonar
          /////////////////////////////////////////
          
          // If the ip address string is set, try to manually connect 
          // to sonar first.
          bool manual_sonar_found = false;
          if (ip_addr_ != "" && ip_addr_ != "0.0.0.0") {
               ret = BVTSonar_Open(son_, "NET", ip_addr_.c_str());
               if( ret != 0 ) {
                    printf("Couldn't find sonar at defined IP address: %s", 
                           ip_addr_.c_str());                    
               } else {
                    manual_sonar_found = true;
               }
          }

          if (!manual_sonar_found) {
               //Create the discovery agent
               BVTSonarDiscoveryAgent agent = BVTSonarDiscoveryAgent_Create();
               if( agent == NULL ) {
                    printf("BVTSonarDiscoverAgent_Create: failed\n");
                    return -1;
               }
          
               // Kick off the discovery process
               ret = BVTSonarDiscoveryAgent_Start(agent);
          
               //Let the discovery process run for a short while (5 secs)
               cout << "Searching for available sonars..." << endl;
               sleep(5);
          
               // See what we found
               int numSonars = 0;
               numSonars = BVTSonarDiscoveryAgent_GetSonarCount(agent);
          
               char SonarIPAddress[20];
          
               for(int i = 0; i < numSonars; i++) {
                    ret = BVTSonarDiscoveryAgent_GetSonarInfo(agent, i, &SonarIPAddress[0], 20);
                    printf("Found Sonar: %d, IP address: %s\n", i, SonarIPAddress);
               }
          
               if(numSonars == 0) {
                    printf("No Sonars Found\n");
                    return -1;
               }    

               // Open the sonar
               //ret = BVTSonar_Open(son_, "NET", "192.168.1.45");
               ret = BVTSonar_Open(son_, "NET", SonarIPAddress);
               if( ret != 0 ) {
                    printf("BVTSonar_Open: ret=%d\n", ret);
                    return -1;
               }
          }
          
     } else {
          /////////////////////////////////////////
          // Reading from sonar file
          /////////////////////////////////////////
          
          // Open the sonar
          ret = BVTSonar_Open(son_, "FILE", fn_.c_str());
          if (ret != 0 ) {
               printf("BVTSonar_Open: ret=%d\n", ret);
               //return 1;
          }
     }

     // Make sure we have the right number of heads_
     heads_ = -1;
     heads_ = BVTSonar_GetHeadCount(son_);
     printf("BVTSonar_GetHeadCount: %d\n", heads_);
	  
     // Get the first head
     head_ = NULL;
     ret = BVTSonar_GetHead(son_, 0, &head_);
     if (ret != 0 ) {
          // Some sonar heads start at 1
          ret = BVTSonar_GetHead(son_, 1, &head_);
          if (ret != 0) {
               printf( "BVTSonar_GetHead: ret=%d\n", ret) ;
               return -1;
          }               
     }
     
     // Check the ping count
     pings_ = -1;
     pings_ = BVTHead_GetPingCount(head_);
     printf("BVTHead_GetPingCount: %d\n", pings_);
     
     // Set the range window
     BVTHead_SetRange(head_, min_range_, max_range_);

     // Build a color mapper
     mapper_ = BVTColorMapper_Create();
     if (mapper_ == NULL) {
          printf("BVTColorMapper_Create: failed\n");
          return -1;
     }

     // Load the bone colormap
     ret = BVTColorMapper_Load(mapper_, color_map_.c_str());
     if(ret != 0) {
          if (color_map_ == "") {
               printf("Color map not set.\n");
          }
          printf("BVTColorMapper_Load: ret=%d\n", ret);          
          return -1;
     }
     
     ////////////////////////////////////////////////////////////////
     // Setup the sonar file logger utilities
     ////////////////////////////////////////////////////////////////
     son_logger_ = BVTSonar_Create();
     if (son_logger_ == NULL) {
          printf("BVTSonar_Create: failed\n");
          return -1;
     }

     initialized_ = true;

     return 0;
#else
     return -1;
#endif
          
}

int Sonar::getNumPings()
{
     return pings_;
}

int Sonar::getCurrentPingNum()
{
     return cur_ping_;
}

void Sonar::setFrameNum(int num)
{
     cur_ping_ = num;
}

int Sonar::reset()
{
     cur_ping_ = 0;
     return 0;
}

int Sonar::SonarLogEnable(bool enable)
{
     if (!enable) {
          // Disable logging
          logging_ = false;
          return 0;
     }

     // Create the sonar file
     cur_log_file_ = save_directory_ + "/" + syllo::get_time_string() + ".son";
     //cur_log_file_ = "/home/syllogismrxs/test.son";

     int ret = BVTSonar_CreateFile(son_logger_, cur_log_file_.c_str(), 
                                   son_, "");
     
     if (ret != 0) {
          printf("BVTSonar_CreateFile: ret=%d\n", ret);
          return -1;
     }
     
     // Get the first head of the file output
     out_head_ = NULL ;
     ret = BVTSonar_GetHead(son_logger_, 0, &out_head_);
     if (ret != 0) {
          printf("BVTSonar_GetHead: ret=%d\n" ,ret);
          return -1;
     }

     logging_ = true;
}

int Sonar::getNextSonarImage(cv::Mat &image)
{
     int status = -1;
     if (mode_ == Sonar::net) {
          status = getSonarImage(image, -1);
     } else if (cur_ping_ < pings_) {
          status = getSonarImage(image, cur_ping_++);
     } else {
          status = 0;
     }
     return status;
}

int Sonar::getSonarImage(cv::Mat &image, int index)
{
#if ENABLE_SONAR == 1

     if (!initialized_) {
          cout << "Sonar wasn't initialized." << endl;
          return -1;
     }

     BVTPing ping = NULL;
     int ret = BVTHead_GetPing(head_, index, &ping);
	  
     if(ret != 0) {
          printf("BVTHead_GetPing: ret=%d\n", ret);
          return 0;
     }
	
     // Logging is enabled, write to file
     if (logging_) {
          ret = BVTHead_PutPing(out_head_, ping);
          if (ret != 0) {
               printf("BVTHead_PutPing: ret=%d\n", ret);
               return -1;
          }
     }     

     ret = BVTPing_GetImage(ping, &img_);
     //ret = BVTPing_GetImageXY(ping, &img_);
     //ret = BVTPing_GetImageRTheta(ping, &img_);
     if (ret != 0) {
          printf("BVTPing_GetImage: ret=%d\n", ret);
          return 0;
     }     

     // Perform the colormapping
     ret = BVTColorMapper_MapImage(mapper_, img_, &cimg_);
     if (ret != 0) {
          printf("BVTColorMapper_MapImage: ret=%d\n", ret);
          return 0;
     }
	
     height_ = BVTColorImage_GetHeight(cimg_);
     width_ = BVTColorImage_GetWidth(cimg_);

     IplImage* sonarImg;
     sonarImg = cvCreateImageHeader(cvSize(width_,height_), IPL_DEPTH_8U, 4);
	
     // And set it's data
     cvSetImageData(sonarImg,  BVTColorImage_GetBits(cimg_), width_*4);
	
     cv::Mat tempImg(sonarImg);
     image = sonarImg;

     cvReleaseImageHeader(&sonarImg);
     BVTPing_Destroy(ping);

     return 1;
#else
     return -1;
#endif
}

int Sonar::width() 
{ 
     return width_;
}

int Sonar::height()
{
     return height_;
}

void Sonar::set_mode(SonarMode_t mode)
{
     mode_ = mode;
}

void Sonar::set_data_mode(DataMode_t data_mode)
{
     data_mode_ = data_mode;
}

void Sonar::set_ip_addr(const std::string &ip_addr)
{
     ip_addr_ = ip_addr;
}

void Sonar::set_input_son_filename(const std::string &fn)
{
     fn_ = fn;
}

void Sonar::set_range(double min_range, double max_range)
{
     min_range_ = min_range;
     max_range_ = max_range;

     BVTHead_SetRange(head_, min_range_, max_range_);
}

void Sonar::set_min_range(double min_range)
{
     this->set_range(min_range, max_range_);     
}

void Sonar::set_max_range(double max_range)
{
     this->set_range(min_range_, max_range);     
}

void Sonar::set_color_map(const std::string &color_map)
{
     color_map_ = color_map;
}

void Sonar::set_save_directory(const std::string &save_directory)
{
     save_directory_ = save_directory;
}
