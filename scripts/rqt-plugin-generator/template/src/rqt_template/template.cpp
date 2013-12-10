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

#include <rqt_(>>>APP-NAME<<<)/(>>>APP-NAME<<<).h>

#include <QMessageBox>
#include <QPainter>

using std::cout;
using std::endl;

namespace rqt_(>>>APP-NAME<<<) {

     (>>>APP-NAME<<<)::(>>>APP-NAME<<<)()
          : rqt_gui_cpp::Plugin()
          , widget_(0)
     {
          setObjectName("(>>>APP-NAME<<<)");
     }

     void (>>>APP-NAME<<<)::initPlugin(qt_gui_cpp::PluginContext& context)
     {
          widget_ = new QWidget();
          ui_.setupUi(widget_);

          if (context.serialNumber() > 1)
          {
               widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
          }
          context.addWidget(widget_);

          // Enable / disable Heading/Depth checkboxes
          connect(ui_.desired_heading_checkbox, SIGNAL(toggled(bool)), this, SLOT(onEnableDesiredHeading(bool)));
          
          // Connect change of value in double spin box to function call
          //connect(ui_.desired_heading_double_spin_box, SIGNAL(valueChanged(double)), this, SLOT(onDesiredHeadingChanged(double)));
          connect(ui_.set_heading_button, SIGNAL(clicked(bool)), this, SLOT(onSetHeading(bool)));
          
          // Create publish and subscriber example
          this->publisher_ = getNodeHandle().advertise<std_msgs::String>("HELLO_WORLD", 1000);
          this->subscriber_ = getNodeHandle().subscribe<std_msgs::Int32>("WRITE_HERE", 1, &(>>>APP-NAME<<<)::callbackNum, this);         
     }
     

     bool (>>>APP-NAME<<<)::eventFilter(QObject* watched, QEvent* event)
     {
          return QObject::eventFilter(watched, event);
     }

     void (>>>APP-NAME<<<)::shutdownPlugin()
     {
          subscriber_.shutdown();
          publisher_.shutdown();
     }
     
     void (>>>APP-NAME<<<)::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
     {
          instance_settings.setValue("desired_heading", ui_.desired_heading_double_spin_box->value());
     }

     void (>>>APP-NAME<<<)::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
     {
          double desired_heading = instance_settings.value("desired_heading", ui_.desired_heading_double_spin_box->value()).toDouble();
          ui_.desired_heading_double_spin_box->setValue(desired_heading);
          
     }
     
     void (>>>APP-NAME<<<)::onSetHeading(bool checked)
     {
          std::ostringstream str;
          str << ui_.desired_heading_double_spin_box->value();
          
          std_msgs::String msg;
          msg.data = "Desired Heading set: " + str.str();
          publisher_.publish(msg);
          
          std::cout << "Heading: "<< msg.data << std::endl;          
     }

     void (>>>APP-NAME<<<)::onEnableDesiredHeading(bool checked)
     {
          ui_.desired_heading_double_spin_box->setEnabled(checked);
          ui_.set_heading_button->setEnabled(checked);          
     }

     void (>>>APP-NAME<<<)::callbackNum(const std_msgs::Int32ConstPtr& msg)
     {
          cout << "Received: " << msg->data << endl;
     }     
}

PLUGINLIB_EXPORT_CLASS(rqt_(>>>APP-NAME<<<)::(>>>APP-NAME<<<), rqt_gui_cpp::Plugin)
