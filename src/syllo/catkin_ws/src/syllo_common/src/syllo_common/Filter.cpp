#include <iostream>
#include "syllo_common/Filter.h"

using std::cout;
using std::endl;

double invert_sign(double input)
{
     return -1*input;
}

double saturate(double input, const double &min, const double &max)
{
     if (min > max) {
          cout << "saturate(): Invalid Min / Max Combo" << endl;
          return 0;
     } else if (input < min) {
          input = min;
     } else if(input > max) {
          input = max;
     }
     return input;
}

double normalize(double input, const double &in_min, const double &in_max,
                 const double &out_min, const double &out_max)
{
     input = saturate(input, in_min, in_max);

     if (in_min >= in_max || out_min >= out_max) {
          cout << "normalize(): Invalid Min / Max Combo" << endl;
          return 0;
     }

     double ratio = input / (in_max - in_min);
     return ratio * (out_max - out_min);

     return input;
}

double norm_degrees(double input)
{
     if (input < 0) {
          input += 360;
     } else if(input >= 360) {
          input -= 360;
     }
     return input;
}
