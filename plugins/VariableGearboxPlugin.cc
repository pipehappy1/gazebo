/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <boost/bind.hpp>
#include <functional>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include "VariableGearboxPlugin.hh"

namespace gazebo
{
class VariableGearboxPluginPrivate
{
  /// \brief Parent model pointer.
  public: physics::ModelPtr model;

  /// \brief Gearbox joint.
  public: physics::JointPtr gearbox;

  /// \brief Input joint.
  public: physics::JointPtr inputJoint;

  /// \brief World update connection.
  public: event::ConnectionPtr updateConnection;
};

/////////////////////////////////////////////////
VariableGearboxPlugin::VariableGearboxPlugin()
  : dataPtr(new VariableGearboxPluginPrivate)
{
}

/////////////////////////////////////////////////
VariableGearboxPlugin::~VariableGearboxPlugin()
{
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  this->dataPtr->model = _parent;

  const std::string jointName = "gearbox_demo";
  auto joint = this->dataPtr->model->GetJoint(jointName);
  if (joint == nullptr || !joint->HasType(physics::Base::GEARBOX_JOINT))
  {
    gzerr << "Could not find a joint named " << jointName << std::endl;
    return;
  }
  this->dataPtr->gearbox = joint;

  auto parentLink = joint->GetParent();
  if (parentLink == nullptr)
  {
    gzerr << "Could not find parent link." << std::endl;
    return;
  }
  std::cerr << "Checking " << parentLink->GetScopedName()
            << " for its joints."
            << std::endl;

  {
    auto joints = parentLink->GetParentJoints();
    if (joints.size() != 1)
    {
      gzerr << "link [" << parentLink->GetScopedName()
            << "] is child of more than 1 joint, not sure which one to pick."
            << std::endl;
      return;
    }
    this->dataPtr->inputJoint = joints.front();
  }
  std::cerr << "Using " << this->dataPtr->inputJoint->GetScopedName()
            << " as input joint."
            << std::endl;

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VariableGearboxPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  const double ref = 1.1;
  double ratio;
  double ref1;
  double ref2;
  double inputAngle = this->dataPtr->inputJoint->GetAngle(0).Radian();
  if (inputAngle > ref)
  {
    ratio = 2;
    ref1 = ref;
    ref2 = -ref;
  }
  else
  {
    ratio = 1;
    ref1 = 0;
    ref2 = 0;
  }

  this->dataPtr->gearbox->SetParam("reference_angle1", 0, ref1);
  this->dataPtr->gearbox->SetParam("reference_angle2", 0, ref2);
  //this->dataPtr->gearbox->SetParam("ratio", 0, ratio);
  std::cerr << "input " << inputAngle << " : " << ratio << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(VariableGearboxPlugin)
}
