#ifndef SYLLONODE_H_
#define SYLLONODE_H_
/// ----------------------------------------------------------------------------
/// @file SylloNode.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-10 15:46:31 yellowin>
///
/// @version 1.0
/// Created: 04 Sep 2013
///
/// ----------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2013 Kevin DeMarco
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
/// The SylloNode class ...
/// 
/// ----------------------------------------------------------------------------
#include "ros/ros.h"

class SylloNode {
private:

     int tick_rate_;     
     ros::Rate *ros_tick_rate_;
     std::string node_name_;
     std::string namespace_;

protected:
public:
     SylloNode();

     void set_node_name(const std::string &node_name);
     const std::string & node_name();

     int init();
     int spin();
     int cleanup();
};

#endif
