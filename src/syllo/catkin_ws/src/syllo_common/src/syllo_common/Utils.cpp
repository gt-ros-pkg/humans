#include <iostream>
#include <stdio.h>
#include <time.h>

#include <syllo_common/Utils.h>

using std::cout;
using std::endl;

#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

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

     std::string get_current_directory()
     {
          char cCurrentPath[512];
               
          if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))) {
               cout << "Failed to get current working directory" << endl;
               return "";
          }

          cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */

          //printf ("The current working directory is %s", cCurrentPath);
          std::string result = cCurrentPath;
          return result;
     }
}
