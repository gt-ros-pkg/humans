#ifndef ORIENTATION_H_
#define ORIENTATION_H_
/// ----------------------------------------------------------------------------
/// @file Orientation.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-10 16:16:56 yellowin>
///
/// @version 1.0
/// Created: 09 Sep 2013
///
/// ----------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ----------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The Orientation class ...
/// 
/// ----------------------------------------------------------------------------

namespace syllo
{
     const double PI = 3.14;
}

double saturate(double input, const double &min, const double &max);

double normalize(double input, const double &in_min, const double &in_max,
                 const double &out_min, const double &out_max);

double norm_degrees(double input);

void quaternionToEuler_q(const double &q0, const double &q1, 
                         const double &q2, const double &q3,
                         double &roll, double &pitch, double &yaw);

void quaternionToEuler_xyzw(const double &x, const double &y, 
                            const double &z, const double &w,
                            double &roll, double &pitch, double &yaw);

void quaternionToEuler_xyzw_deg(const double &x, const double &y, 
                                const double &z, const double &w,
                                double &roll, double &pitch, double &yaw);

void eulerToQuaternion_q(const double &roll, const double &pitch, 
                       const double &yaw,
                       double &q0, double &q1, 
                       double &q2, double &q3);

void eulerToQuaternion_xyzw(const double &roll, const double &pitch, 
                            const double &yaw,
                            double &x, double &y, 
                            double &z, double &w);

void eulerToQuaternion_xyzw_deg(const double &roll, const double &pitch, 
                                const double &yaw,
                                double &x, double &y, 
                                double &z, double &w);


#endif
