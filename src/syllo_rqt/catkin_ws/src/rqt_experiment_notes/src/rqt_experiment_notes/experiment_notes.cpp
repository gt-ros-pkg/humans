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

#include <rqt_experiment_notes/experiment_notes.h>

#include <QMessageBox>
#include <QPainter>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <stdio.h>
#include <time.h> 

using std::cout;
using std::endl;

namespace rqt_experiment_notes {

     experiment_notes::experiment_notes()
          : rqt_gui_cpp::Plugin()
          , widget_(0)
     {
          setObjectName("experiment_notes");
     }

     void experiment_notes::initPlugin(qt_gui_cpp::PluginContext& context)
     {
          widget_ = new QWidget();
          ui_.setupUi(widget_);

          if (context.serialNumber() > 1)
          {
               widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
          }
          context.addWidget(widget_);

          // Enable / disable Heading/Depth checkboxes
          //connect(ui_.desired_heading_checkbox, SIGNAL(toggled(bool)), this, SLOT(onEnableDesiredHeading(bool)));
          connect(ui_.clear_pushButton, SIGNAL(released()), this, SLOT(onClearText()));
          connect(ui_.save_pushButton, SIGNAL(released()), this, SLOT(onSave()));
          
          // Connect change of value in double spin box to function call
          //connect(ui_.desired_heading_double_spin_box, SIGNAL(valueChanged(double)), this, SLOT(onDesiredHeadingChanged(double)));
          //connect(ui_.set_heading_button, SIGNAL(clicked(bool)), this, SLOT(onSetHeading(bool)));
          
          // Create publish and subscriber example
          this->notes_pub_ = getNodeHandle().advertise<std_msgs::String>("experiment_notes", 10);
          //this->subscriber_ = getNodeHandle().subscribe<std_msgs::Int32>("WRITE_HERE", 1, &experiment_notes::callbackNum, this);         

          // Determine home directory
          struct passwd *pw = getpwuid(getuid());
          const char *homedir = pw->pw_dir;

          // Get time string
          time_t rawtime;
          struct tm * timeinfo;
          char time_buf [80];
          time (&rawtime);
          timeinfo = localtime (&rawtime);
          strftime (time_buf,80,"%Y_%m_%d-%H:%M:%S",timeinfo);

          filename_ = std::string(homedir) + "/experiment-" + std::string(time_buf) + ".txt";
          fd_.open(filename_.c_str());
     }
     

     bool experiment_notes::eventFilter(QObject* watched, QEvent* event)
     {
          return QObject::eventFilter(watched, event);
     }

     void experiment_notes::shutdownPlugin()
     {
          //subscriber_.shutdown();
          fd_.close();
          notes_pub_.shutdown();
     }
     
     void experiment_notes::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
     {
          //instance_settings.setValue("desired_heading", ui_.desired_heading_double_spin_box->value());
     }

     void experiment_notes::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
     {
          //double desired_heading = instance_settings.value("desired_heading", ui_.desired_heading_double_spin_box->value()).toDouble();
          //ui_.desired_heading_double_spin_box->setValue(desired_heading);          
     }
     
     void experiment_notes::onClearText()
     {
          ui_.notes->clear();
     }
     void experiment_notes::onSave()
     {         
          std_msgs::String msg;
          msg.data = ui_.notes->toPlainText().toStdString();
          if (msg.data != "") {               
               notes_pub_.publish(msg);
               fd_ << ros::Time::now() << " ";
               fd_ << msg.data << endl;
          }
          ui_.notes->clear();
          //videoray::Notes notes;
          //notes.header.stamp = ros::Time::now();
          //notes.notes = ui_.notes->toPlainText().toStdString();
          //if (notes.notes != "") {
          //     notes_pub_.publish(notes);
          //     fd_ << notes.header.stamp << " ";
          //     fd_ << ui_.notes->toPlainText().toStdString() << endl;
          //}
          //ui_.notes->clear();
     }
}

PLUGINLIB_EXPORT_CLASS(rqt_experiment_notes::experiment_notes, rqt_gui_cpp::Plugin)
