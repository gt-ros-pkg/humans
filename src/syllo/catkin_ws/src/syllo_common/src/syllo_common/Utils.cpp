#include <iostream>
#include <stdio.h>
#include <time.h>

#include <syllo_common/Utils.h>

using std::cout;
using std::endl;

namespace syllo {
     
     std::string get_time_string() 
     {
          time_t rawtime;
          struct tm * timeinfo;
     
          time (&rawtime);
          timeinfo = localtime (&rawtime);

          char buffer [80];
          strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", timeinfo);
          
          std::string result = buffer;
          return result;
     }
}
