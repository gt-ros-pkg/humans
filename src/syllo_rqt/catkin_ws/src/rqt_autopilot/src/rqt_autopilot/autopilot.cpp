/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
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

#include <rqt_autopilot/autopilot.h>

#include <QMessageBox>
#include <QPainter>

using std::cout;
using std::endl;

namespace rqt_autopilot {

     Autopilot::Autopilot()
          : rqt_gui_cpp::Plugin()
          , widget_(0)
     {
          setObjectName("Autopilot");
     }

     void Autopilot::initPlugin(qt_gui_cpp::PluginContext& context)
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
          connect(ui_.desired_depth_checkbox, SIGNAL(toggled(bool)), this, SLOT(onEnableDesiredDepth(bool)));

          // Connect change of value in double spin box to function call
          connect(ui_.set_heading_button, SIGNAL(clicked(bool)), this, SLOT(onSetHeading(bool)));
          connect(ui_.set_depth_button, SIGNAL(clicked(bool)), this, SLOT(onSetDepth(bool)));
          
          // Create publisher
          this->desired_pub_ = getNodeHandle().advertise<videoray::DesiredTrajectory>("desired_trajectory", 1000);
     }
     

     bool Autopilot::eventFilter(QObject* watched, QEvent* event)
     {
          return QObject::eventFilter(watched, event);
     }

     void Autopilot::shutdownPlugin()
     {
          desired_pub_.shutdown();
     }
     
     void Autopilot::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
     {
          instance_settings.setValue("desired_heading", ui_.desired_heading_double_spin_box->value());
          instance_settings.setValue("desired_depth", ui_.desired_depth_double_spin_box->value());
     }

     void Autopilot::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
     {
          double desired_heading = instance_settings.value("desired_heading", ui_.desired_heading_double_spin_box->value()).toDouble();
          ui_.desired_heading_double_spin_box->setValue(desired_heading);

          double desired_depth = instance_settings.value("desired_depth", ui_.desired_depth_double_spin_box->value()).toDouble();
          ui_.desired_depth_double_spin_box->setValue(desired_depth);
     }
     
     void Autopilot::onSetHeading(bool checked)
     {
          // Enable the heading controller
          desired_.heading_enabled = true;
          
          // Get value of heading
          desired_.heading = ui_.desired_heading_double_spin_box->value();
          desired_pub_.publish(desired_);
     }

     void Autopilot::onSetDepth(bool checked)
     {
          // Enable the depth controller
          desired_.depth_enabled = true;

          // Save the value of the desired depth
          desired_.depth = ui_.desired_depth_double_spin_box->value();
          desired_pub_.publish(desired_);
     }

     void Autopilot::onEnableDesiredHeading(bool checked)
     {
          ui_.desired_heading_double_spin_box->setEnabled(checked);
          ui_.set_heading_button->setEnabled(checked);

          if (!checked) {
               // Disable controller
               desired_.heading_enabled = false;
               desired_pub_.publish(desired_);
          }
     }

     void Autopilot::onEnableDesiredDepth(bool checked)
     {
          ui_.desired_depth_double_spin_box->setEnabled(checked);
          ui_.set_depth_button->setEnabled(checked);          

          if (!checked) {
               // Disable controller
               desired_.depth_enabled = false;
               desired_pub_.publish(desired_);
          }
     }     
}

PLUGINLIB_EXPORT_CLASS(rqt_autopilot::Autopilot, rqt_gui_cpp::Plugin)
