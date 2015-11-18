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
#ifndef _IGNITION_FRAMEGRAPH_HH_
#define _IGNITION_FRAMEGRAPH_HH_

#include <memory>
#include <list>
#include <ignition/math/Frame.hh>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declare private data
    class FrameGraphPrivate;

    /// \def Frame_L
    /// \brief List of frame name and pose pairs.
    /// \sa Init()
    using Frame_L = std::list<std::pair<std::string, Pose3d>>;

    /// \def Edge_L
    /// \brief List of edge name pairs (parent, child).
    /// \sa Init()
    using Edge_L = std::list<std::pair<std::string, std::string>>;

    /// \class FrameGraph FrameGraph.hh ignition/math/FrameGraph.hh
    /// \brief The FrameGraph class repsents of set of connected Frames.
    /// Function within FrameGraph allow construction of transforms between
    /// frames.
    class IGNITION_VISIBLE FrameGraph
    {
      /// \brief Constructor
      public: FrameGraph();

      /// \brief Create a graph using a list of frames and edges.
      ///
      /// Example usage:
      ///  FrameGraph graph(
      ///   {
      ///     {"a", Pose3d::Zero},
      ///     {"b", Pose3d::Zero},
      ///     {"c", Pose3d::Zero}
      ///   },
      ///   {
      ///     {"a", "b"},
      ///     {"a", "c"},
      ///     {"c", "b"}
      ///   });
      ///
      /// \param[in] _frame List of frames and their values.
      /// \param[in] _edges List of edges.
      public: FrameGraph(const Frame_L &_frames, const Edge_L &_edges);

      /// \brief Destructor
      public: virtual ~FrameGraph();

      /// \brief Get a node (frame) by name
      /// \param[in] _name Name of the node.
      /// \return Weak pointer to the node.
      public: FrameWeakPtr Node(const std::string &_name) const;

      /// \brief Get the index of a node.
      /// \param[in] _name Name of the node to find.
      /// \return Index of the node. -1 is returned when the node is not
      /// found.
      public: int NodeIndex(const std::string &_name) const;

      /// \brief Initialize a graph using a list of frames and edges.
      /// \param[in] _frame List of frames and their values.
      /// \param[in] _edges List of edges.
      public: void Init(const Frame_L &_frames, const Edge_L &_edges);

      /// \brief Add a new node (frame).
      /// \param[in] _name Name of the node.
      /// \param[in] _pose Pose the node.
      /// \return Pointer to the new node.
      public: FrameWeakPtr AddNode(const std::string &_name,
                                   const Pose3d &_pose);

      /// \brief Add a new node.
      /// \param[in] _node Pointer to the node to add.
      public: void AddNode(FramePtr _frame);

      /// \brief Add a new edge.
      /// \param[in] _parent Pointer to the parent node
      /// \param[in] _child Pointer to the child node
      public: void AddEdge(FramePtr _parent, FramePtr _child);

      /// \brief Compute the transform from a start frame to an end frame.
      /// \param[in] _start Name of the start frame.
      /// \param[in] _end Name of the end frame.
      /// \return Transform from the start frame to end frame.
      public: Pose3d Transform(const std::string &_start,
                               const std::string &_end);

      /// \brief Get the shortest path from start to end using Dijstra's
      /// algorithm.
      /// \param[in] _start Name of the start node
      /// \param[in] _end Name of the end node
      public: void Path(const std::string &_start, const std::string &_end,
                        std::list<FrameWeakPtr> &_result);

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _pt Graph to output
      /// \return the stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const ignition::math::FrameGraph &_g)
      {
        _g.Print(_out);
        return _out;
      }

      /// \brief Helper function to print a graph
      /// \param[in] _out Output stream
      private: void Print(std::ostream &_out) const;

      /// \internal
      /// \brief Private data structure.
      private: std::unique_ptr<FrameGraphPrivate> dataPtr;
    };
  }
}
#endif
