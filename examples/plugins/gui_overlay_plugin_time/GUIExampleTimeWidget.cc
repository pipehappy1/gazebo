/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "GUIExampleTimeWidget.hh"
#include "gazebo/gui/GuiIface.hh"

using namespace gazebo;
using namespace gui;
/*
MyExample::~MyExample()
{
}

void MyExample::Load(sdf::ElementPtr _elem)
{
  printf("Loaded\n");
  gui::get_active_camera();
}

Q_EXPORT_PLUGIN2(myexample, gazebo::gui::MyExample)
  */

#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "GUIExampleTimeWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GUIExampleTimeWidget::~GUIExampleTimeWidget()
{
}

/////////////////////////////////////////////////
void GUIExampleTimeWidget::Load(sdf::ElementPtr _elem)
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *frameLayout = new QHBoxLayout();

  QLabel *label = new QLabel(tr("Sim Time:"));

  // Create a time label
  QLabel *timeLabel = new QLabel(tr("00:00:00.00"));

  // Add the label to the frame's layout
  frameLayout->addWidget(label);
  frameLayout->addWidget(timeLabel);
  connect(this, SIGNAL(SetSimTime(QString)),
      timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(200, 10);
  this->resize(200, 30);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->statsSub = this->node->Subscribe("~/world_stats",
      &GUIExampleTimeWidget::OnStats, this);

  /*printf("A Thread[%ld]\n", pthread_self());
  this->cam = gui::get_active_camera();
  printf("B\n");
  if (!this->cam)
    printf("Camera is NULL\n");
    */

  this->show();
}

/////////////////////////////////////////////////
void GUIExampleTimeWidget::OnStats(ConstWorldStatisticsPtr &_msg)
{
  printf("On Stats\n");
  this->SetSimTime(QString::fromStdString(
        this->FormatTime(_msg->sim_time())));
}

/////////////////////////////////////////////////
std::string GUIExampleTimeWidget::FormatTime(const msgs::Time &_msg) const
{
  std::ostringstream stream;
  unsigned int day, hour, min, sec, msec;

  stream.str("");

  sec = _msg.sec();

  day = sec / 86400;
  sec -= day * 86400;

  hour = sec / 3600;
  sec -= hour * 3600;

  min = sec / 60;
  sec -= min * 60;

  msec = rint(_msg.nsec() * 1e-6);

  stream << std::setw(2) << std::setfill('0') << day << " ";
  stream << std::setw(2) << std::setfill('0') << hour << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  return stream.str();
}

Q_EXPORT_PLUGIN2(gui_example_time_widget, gazebo::gui::GUIExampleTimeWidget)
