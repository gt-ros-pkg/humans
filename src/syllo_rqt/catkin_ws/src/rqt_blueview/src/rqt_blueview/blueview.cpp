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
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <rqt_blueview/blueview.h>

#include <QMessageBox>
#include <QPainter>

using std::cout;
using std::endl;

namespace rqt_blueview {

     blueview::blueview()
          : rqt_gui_cpp::Plugin()
          , widget_(0)
     {
          setObjectName("blueview");
     }

     void blueview::initPlugin(qt_gui_cpp::PluginContext& context)
     {
          widget_ = new QWidget();
          ui_.setupUi(widget_);

          if (context.serialNumber() > 1)
          {
               widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
          }
          context.addWidget(widget_);
          
          connect(ui_.min_dist_slider, SIGNAL(sliderMoved(int)), this, SLOT(onMinDistSliderChanged(int)));
          connect(ui_.min_dist_spinbox, SIGNAL(valueChanged(double)), this, SLOT(onMinDistSpinboxChanged(double)));
          
          connect(ui_.max_dist_slider, SIGNAL(sliderMoved(int)), this, SLOT(onMaxDistSliderChanged(int)));
          connect(ui_.max_dist_spinbox, SIGNAL(valueChanged(double)), this, SLOT(onMaxDistSpinboxChanged(double)));
          
          connect(ui_.thresh_slider, SIGNAL(sliderMoved(int)), this, SLOT(onThreshDistSliderChanged(int)));
          connect(ui_.thresh_spinbox, SIGNAL(valueChanged(double)), this, SLOT(onThreshDistSpinboxChanged(double)));
          
          connect(ui_.log_checkbox, SIGNAL(stateChanged(int)), this, SLOT(onLogChecked(int)));

          //// Enable / disable Heading/Depth checkboxes
          //connect(ui_.desired_heading_checkbox, SIGNAL(toggled(bool)), this, SLOT(onEnableDesiredHeading(bool)));
          //
          //// Connect change of value in double spin box to function call
          ////connect(ui_.desired_heading_double_spin_box, SIGNAL(valueChanged(double)), this, SLOT(onDesiredHeadingChanged(double)));
          //connect(ui_.set_heading_button, SIGNAL(clicked(bool)), this, SLOT(onSetHeading(bool)));
          //
          //// Create publish and subscriber example
          this->max_range_pub_ = getNodeHandle().advertise<std_msgs::Float32>("sonar_max_range", 1);
          this->min_range_pub_ = getNodeHandle().advertise<std_msgs::Float32>("sonar_min_range", 1);
          this->enable_log_pub_ = getNodeHandle().advertise<std_msgs::Bool>("sonar_enable_log", 1);
          this->thresh_pub_ = getNodeHandle().advertise<std_msgs::Float32>("sonar_thresh", 1);
          //this->subscriber_ = getNodeHandle().subscribe<std_msgs::Int32>("WRITE_HERE", 1, &blueview::callbackNum, this);         
     }
     

     bool blueview::eventFilter(QObject* watched, QEvent* event)
     {
          return QObject::eventFilter(watched, event);
     }

     void blueview::shutdownPlugin()
     {
          max_range_pub_.shutdown();
          min_range_pub_.shutdown();
          enable_log_pub_.shutdown();
          thresh_pub_.shutdown();
     }
     
     void blueview::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
     {
          //instance_settings.setValue("desired_heading", ui_.desired_heading_double_spin_box->value());
     }

     void blueview::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
     {
          //double desired_heading = instance_settings.value("desired_heading", ui_.desired_heading_double_spin_box->value()).toDouble();
          //ui_.desired_heading_double_spin_box->setValue(desired_heading);
          
     }
     
     //void blueview::onSetHeading(bool checked)
     //{
     //     //std::ostringstream str;
     //     //str << ui_.desired_heading_double_spin_box->value();
     //     //
     //     //std_msgs::String msg;
     //     //msg.data = "Desired Heading set: " + str.str();
     //     //publisher_.publish(msg);
     //     //
     //     //std::cout << "Heading: "<< msg.data << std::endl;          
     //}

     void blueview::onMinDistSliderChanged(int value)
     {
          ui_.min_dist_spinbox->setValue(value);          
     }

     void blueview::onMinDistSpinboxChanged(double value)
     {
          ui_.min_dist_slider->setValue((int)value);

          std_msgs::Float32 msg;
          msg.data = value;
          min_range_pub_.publish(msg);
     }

     void blueview::onMaxDistSliderChanged(int value)
     {
          ui_.max_dist_spinbox->setValue(value);
     }

     void blueview::onMaxDistSpinboxChanged(double value)
     {
          ui_.max_dist_slider->setValue((int)value);

          std_msgs::Float32 msg;
          msg.data = value;
          max_range_pub_.publish(msg);
     }

     void blueview::onThreshDistSliderChanged(int value)
     {
          ui_.thresh_spinbox->setValue(value);
     }

     void blueview::onThreshDistSpinboxChanged(double value)
     {
          ui_.thresh_slider->setValue((int)value);

          std_msgs::Float32 msg;
          msg.data = value;
          thresh_pub_.publish(msg);
     }

     void blueview::onLogChecked(int enable)
     {
          std_msgs::Bool msg;
          if (enable) {               
               msg.data = true;
               
          } else {
               msg.data = false;
          }
          enable_log_pub_.publish(msg);
     }

     void blueview::callbackNum(const std_msgs::Int32ConstPtr& msg)
     {
          cout << "Received: " << msg->data << endl;
     }     
}

PLUGINLIB_EXPORT_CLASS(rqt_blueview::blueview, rqt_gui_cpp::Plugin)
