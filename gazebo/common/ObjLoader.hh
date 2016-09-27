/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_COMMON_OBJLOADER_HH_
#define GAZEBO_COMMON_OBJLOADER_HH_

#include <memory>

#include "gazebo/common/MeshLoader.hh"
#include "gazebo/util/system.hh"

class TiXmlElement;

namespace gazebo
{
  namespace common
  {
    // class Material;
    class ObjLoaderPrivate;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ObjLoader ObjLoader.hh common/common.hh
    /// \brief Class used to load obj mesh files
    class GZ_COMMON_VISIBLE ObjLoader : public MeshLoader
    {
      /// \brief Constructor
      public: ObjLoader();

      /// \brief Destructor
      public: virtual ~ObjLoader();

      /// \brief Load a mesh
      /// \param[in] _filename Collada file to load
      /// \return Pointer to a new Mesh
      public: virtual Mesh *Load(const std::string &_filename);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ObjLoaderPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
