/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_MATH_FLUIDADDEDMASS_HH_
#define GZ_MATH_FLUIDADDEDMASS_HH_

#include <algorithm>
#include <array>

#include <gz/math/config.hh>
#include <gz/math/Helpers.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class FluidAddedMass FluidAddedMass.hh gz/math/FluidAddedMass.hh
    /// \brief TODO
    template<typename T>
    class FluidAddedMass
    {
      /// \brief Size of matrix is fixed to 6x6
      public: static constexpr std::size_t MatrixSize{6};

      /// \brief Size of storage for a 6x6 symmetric matrix is its triangular number, 21
      public: static constexpr std::size_t StorageSize = (MatrixSize * (MatrixSize + 1)) / 2;

      /// \brief Default Constructor, which inializes the terms to zero.
      public: FluidAddedMass()
      {
        std::fill(std::begin(this->terms), std::end(this->terms),
            static_cast<T>(0));
      }

      /// \brief Constructor.
      /// TODO
      public: FluidAddedMass(const std::array<T, StorageSize> &_terms)
      : terms(_terms)
      {}

      /// \brief Copy constructor.
      /// \param[in] _m FluidAddedMass element to copy
      public: FluidAddedMass(const FluidAddedMass<T> &_m) = default;

      /// \brief Destructor.
      public: ~FluidAddedMass() = default;

      /// \brief TODO
      public:
      template<class ... Terms>
      void SetTerms(Terms... _terms)
      {
        static_assert(StorageSize == sizeof...(_terms),
            "Wrong number of terms provided.");
        this->terms = {std::forward<Terms>(_terms)...};
      }

      /// \brief TODO
      public: const std::array<T, StorageSize> &Terms() const
      {
        return this->terms;
      }

      /// \brief TODO
      public: void SetTerm(std::size_t _row, std::size_t _col, T _value)
      {
        this->terms[this->RowColToIndex(_row, _col)] = _value;
      }

      /// \brief TODO
      public: T Term(std::size_t _row, std::size_t _col) const
      {
        return this->terms[this->RowColToIndex(_row, _col)];
      }

      /// \brief TODO
      public: std::size_t RowColToIndex(std::size_t _row, std::size_t _col) const
      {
        auto row = clamp(_row, GZ_ZERO_SIZE_T, this->MatrixSize - 1);
        auto col = clamp(_col, GZ_ZERO_SIZE_T, this->MatrixSize - 1);;

        if (row <= col)
           return row * this->MatrixSize - (row - 1) * row / 2 + col - row;
        else
           return col * this->MatrixSize - (col - 1) * col / 2 + row - col;
      }

      /// \brief
      private: std::array<T, StorageSize> terms;
    };

    typedef FluidAddedMass<double> FluidAddedMassd;
    typedef FluidAddedMass<float> FluidAddedMassf;
    }
  }
}
#endif
