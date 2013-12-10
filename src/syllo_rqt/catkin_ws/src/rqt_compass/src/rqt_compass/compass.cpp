/*
 * Copyright (c) 2013, Kevin DeMarco
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <rqt_compass/compass.h>

#include <syllo_common/Orientation.h>

#include <QMessageBox>
#include <QPainter>

using std::cout;
using std::endl;

namespace rqt_compass {

     compass::compass()
          : rqt_gui_cpp::Plugin()
          , widget_(0)
     {
          setObjectName("Compass");
     }
    
     void compass::initPlugin(qt_gui_cpp::PluginContext& context)
     {
          widget_ = new QWidget();
          ui_.setupUi(widget_);

          if (context.serialNumber() > 1)
          {
               widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
          }
          context.addWidget(widget_);
          
          
          rose_ = new QwtSimpleCompassRose( 4, 1 );
          needle_ = new QwtCompassMagnetNeedle();
          ui_.Compass->setRose(rose_);
          ui_.Compass->setNeedle(needle_);
                    
          // Create subscriber
          this->subscriber_ = getNodeHandle().subscribe<geometry_msgs::PoseStamped>("/pose", 1, &compass::callback_pose, this);
     }
     

     bool compass::eventFilter(QObject* watched, QEvent* event)
     {
          return QObject::eventFilter(watched, event);
     }

     void compass::shutdownPlugin()
     {              
          subscriber_.shutdown();
     }
     
     void compass::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
     {
          //instance_settings.setValue("desired_heading", ui_.desired_heading_double_spin_box->value());
     }

     void compass::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
     {
          //double desired_heading = instance_settings.value("desired_heading", ui_.desired_heading_double_spin_box->value()).toDouble();
          //ui_.desired_heading_double_spin_box->setValue(desired_heading);          
     }
     
     void compass::onSetHeading(bool checked)
     {
          //std::ostringstream str;
          //str << ui_.desired_heading_double_spin_box->value();
          //
          //std_msgs::String msg;
          //msg.data = "Desired Heading set: " + str.str();
          //publisher_.publish(msg);
          //
          //std::cout << "Heading: "<< msg.data << std::endl;          
     }

     void compass::onEnableDesiredHeading(bool checked)
     {
          //ui_.desired_heading_double_spin_box->setEnabled(checked);
          //ui_.set_heading_button->setEnabled(checked);          
     }

     void compass::callback_pose(const geometry_msgs::PoseStampedConstPtr& msg)
     {
          geometry_msgs::Quaternion orientation = msg->pose.orientation;

          double roll, pitch, yaw;
          
          quaternionToEuler_xyzw_deg(orientation.x, orientation.y, 
                                 orientation.z, orientation.w,
                                 roll, pitch, yaw);

          ui_.Compass->setValue(yaw);
          ui_.heading_spinbox->setValue(yaw);
     }     
}

PLUGINLIB_EXPORT_CLASS(rqt_compass::compass, rqt_gui_cpp::Plugin)
