#include <iostream>
#include "ros/ros.h"
#include "syllo_common/SylloNode.h"

using std::cout;
using std::endl;

SylloNode::SylloNode()
{
}

int SylloNode::init()
{
     namespace_ = ros::this_node::getNamespace();
     node_name_ = ros::this_node::getName();     

     ros::param::param<int>("tick_rate", tick_rate_, 100);
     ros_tick_rate_ = new ros::Rate(tick_rate_);
     
     cout << "================================" << endl;
     cout << "               _ _         " << endl; 
     cout << "     ___ _   _| | | ___    " << endl;
     cout << "    / __| | | | | |/ _ \\  " << endl;
     cout << "    \\__ \\ |_| | | | (_) |" << endl;
     cout << "    |___/\\__, |_|_|\\___/ " << endl; 
     cout << "         |___/             " << endl;
     cout << "================================" << endl;
     cout << "Node Name: " << node_name_ << endl;
     cout << "Tick Rate: " << tick_rate_ << endl;              

     return 0;
}

int SylloNode::spin()
{
     ros::spinOnce();
     ros_tick_rate_->sleep();
     
     return 0;
}

int SylloNode::cleanup()
{
     if (ros_tick_rate_) {
          delete ros_tick_rate_;
     }
     return 0;
}
