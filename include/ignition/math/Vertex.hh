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
#ifndef IGNITION_MATH_VERTEX_HH_
#define IGNITION_MATH_VERTEX_HH_

#include <memory>
#include <string>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    /// \class Vertex Vertex.hh ignition/math/Vertex.hh
    /// \brief A vertex in a graph.
    template<typename T>
    class IGNITION_VISIBLE Vertex
    {
      public: static const Vertex<T> Nan;

      /// \brief Constructor
      public: Vertex(): dataPtr(new VertexPrivate<T>) {}

      /// \brief Copy constructor
      /// \param[in] _vertex Vertex to copy
      public: Vertex(const Vertex<T> &_vertex)
              : dataPtr(new VertexPrivate<T>)
      {
        this->dataPtr->name = _vertex.Name();
        this->dataPtr->data = _vertex.Data();
      }

      /// \brief Constructor
      /// \param[in] _name Name of the vertex
      public: Vertex(const std::string &_name)
              : dataPtr(new VertexPrivate<T>)
      {
       this->dataPtr->name = _name;
      }

      /// \brief Constructor
      /// \param[in] _name Name of the vertex
      /// \param[in] _data Data held by this vertex
      public: Vertex(const std::string &_name, const T &_data)
              : dataPtr(new VertexPrivate<T>(_name, _data))
      {
      }

      /// \brief Destructor.
      public: virtual ~Vertex()
      {
      }

      /// \brief Get the name of the vertex.
      /// \return Name of the vertex.
      public: std::string Name() const
      {
        return this->dataPtr->name;
      }

      /// \brief Set the name of the vertex.
      /// \param[in] _name Name of the vertex.
      public: void SetName(const std::string &_name)
      {
        this->dataPtr->name = _name;
      }

      /// \brief Get the data held by this vertex.
      /// \return Map of the data in this vertex.
      public: const T &Data() const
      {
        return this->dataPtr->data;
      }

      /// \brief Set the data held by this vertex.
      /// \param[in] _data Data to copy.
      public: void SetData(const T &_data) const
      {
        this->dataPtr->data = _data;
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _pt Vertex to output
      /// \return the stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const ignition::math::Vertex<T> &_n)
      {
        _out << _n.Name() << " = " << _n.Data();
        return _out;
      }

      /// \brief Private data for the Vertex class
      private: template<typename U>
               class VertexPrivate
      {
        /// Constructor
        public: VertexPrivate() {}

        /// Constructor
        /// \param[in] _name Name of the vertex
        /// \param[in] _data Data held by the vertex
        public: VertexPrivate(const std::string &_name,
                              const U &_data)
                 : name(_name), data(_data) {}

         /// \brief Vertex's name
        public: std::string name = "";

        /// \brief Vertex's data
        public: U data;
      };

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<VertexPrivate<T>> dataPtr;
    };

    template<typename T> const Vertex<T> Vertex<T>::Nan("__inf__");
  }
}
#endif
