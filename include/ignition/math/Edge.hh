/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_EDGE_HH_
#define IGNITION_MATH_EDGE_HH_

#include <string>
#include <memory>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    /// \class Edge Edge.hh ignition/math/Edge.hh
    /// \brief An edge in a graph.
    template<typename T>
    class IGNITION_VISIBLE Edge
    {
      public: static const Edge<T> Nan;

      /// \brief Constructor
      public: Edge(): dataPtr(new EdgePrivate<T>) {}

      /// \brief Copy constructor
      /// \param[in] _edge Edge to copy
      public: Edge(const Edge<T> &_edge)
              : Edge()
      {
        this->dataPtr->name = _node.Name();
        this->dataPtr->parent = _node.Parent();
        this->dataPtr->child = _node.Child();
        this->dataPtr->data = _node.Data();
      }

      /// \brief Constructor
      /// \param[in] _name Name of the edge
      /// \param[in] _parent Name of the parent node
      /// \param[in] _child Name of the child node
      public: Edge(const std::string &_name
                   const std::string &_parent = "",
                   const std::string &_child = "")
              : dataPtr(new EdgePrivate<T>(_name, _parent, _child)
      {
      }

      /// \brief Constructor
      /// \param[in] _name Name of the edge
      /// \param[in] _data Data held by this edge
      public: Edge(const std::string &_name, const std::string &_parent,
                   const std::string &_child, const T &_data)
              : dataPtr(new EdgePrivate<T>(_name, _parent, _child, _data))
      {
      }

      /// \brief Destructor.
      public: virtual ~Edge()
      {
      }

      /// \brief Get the name of the node.
      /// \return Name of the node.
      public: std::string Name() const
      {
        return this->dataPtr->name;
      }

      /// \brief Set the name of the node.
      /// \param[in] _name Name of the node.
      public: void SetName(const std::string &_name)
      {
        this->dataPtr->name = _name;
      }

      /// \brief Get the parent name of the edge.
      /// \return Name of the parent node.
      public: std::string Parent() const
      {
        return this->dataPtr->parent;
      }

      /// \brief Set the parent name of the edge.
      /// \param[in] _name Name of the parent node.
      public: void SetParent(const std::string &_name)
      {
        this->dataPtr->parent = _name;
      }

      /// \brief Get the child name of the edge.
      /// \return Name of the child node.
      public: std::string Child() const
      {
        return this->dataPtr->child;
      }

      /// \brief Set the child name of the edge.
      /// \param[in] _name Name of the child node.
      public: void SetChild(const std::string &_name)
      {
        this->dataPtr->child = _name;
      }

      /// \brief Private data for the Edge class
      private: template<typename U>
               class EdgePrivate
      {
        /// Constructor
        public: EdgePrivate() {}

        /// Constructor
        /// \param[in] _name Name of the edge
        /// \param[in] _data Data held by the node
        public: EdgePrivate(const std::string &_name, const U &_data)
                 : name(_name), data(_data) {}

         /// \brief Edge's name
        public: std::string name = "";

        /// \brief Edge's pose
        public: U data;
      };

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<EdgePrivate<T>> dataPtr;
    };

    template<typename T> const Edge<T> Edge<T>::Nan("__inf__");
  }
}
#endif
