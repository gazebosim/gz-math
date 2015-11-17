/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _IGNITION_PATH_PRIVATE_HH_
#define _IGNITION_PATH_PRIVATE_HH_

#include <string>
#include <vector>

namespace ignition
{
  namespace math
  {
    /// \brief A utility class to parse a path like "/world/sphere/center" into
    /// its components. Paths can be absolute or relative (i.e "../left/right")
    class PathPrivate
    {
      /// \brief Constructor
      /// \param[in] _path The path
      public: PathPrivate(const std::string &_path);

      /// \brief Access to the path elements
      /// \return A vector of path elements
      public: const std::vector<std::string> &Elems() const;

      /// \brief Access the path
      /// \return The path in a single string (string passed in the constructor)
      public: std::string Path() const;

      /// \brief Checks if the path is absolute (starts with "/world")
      /// \return True if it is a full path, false if the path is relative
      public: bool IsAbsolute() const;

      /// \brief Checks if a string is a valid path element. It must contain
      /// no space, punctuation or special characters
      /// \param[in] _name The path element to verify
      /// \return True if the name is valid
      public: static bool CheckName(const std::string &_name);

      /// \brief The path string to be parsed
      private: std::string path;

      /// \brief The elements of the path
      private: std::vector<std::string> pathElems;
    };
  }
}
#endif
