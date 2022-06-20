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
#ifndef GZ_MATH_MATRIXX_HH_
#define GZ_MATH_MATRIXX_HH_

#include <cassert>
#include <utility>
#include <ignition/math/Helpers.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class MatrixX MatrixX.hh ignition/math/MatrixX.hh
    /// \brief A matrix that can have any given size.
    /// \tparam Precision Precision such as int, double or float.
    /// \tparam RowCount Number of rows in the matrix, must be higher than zero.
    /// \tparam ColCount Number of columns in the matrix, must be higher than
    /// zero.
    template<typename Precision, std::size_t RowCount,
        std::size_t ColCount = RowCount>
    class MatrixX
    {
      /// \brief Constructor
      public: MatrixX()
      {
        static_assert(RowCount > 0 && ColCount > 0,
            "Matrix can't have zero rows or columns.");

        for (std::size_t row = 0; row < RowCount; ++row)
        {
          for (std::size_t col = 0; col < ColCount; ++col)
          {
            this->data[row][col] = static_cast<Precision>(0);
          }
        }
      }

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      public: MatrixX(const MatrixX<Precision, RowCount, ColCount> &_m)
      {
        this->data = _m.data;
      }

      /// \brief Move constructor
      /// \param _m Matrix to move
      public: MatrixX(const MatrixX<Precision, RowCount, ColCount> &&_m)
      {
        this->data = std::move(_m.data);
      }

      /// \brief Constructor
      /// \param[in] _values Values to fill matrix. The total number of passed
      /// values must be equal to RowCount * ColCount.
      public:
      template<class ... Values>
      explicit MatrixX(Values... _values)
      {
        static_assert(RowCount > 0 && ColCount > 0,
            "Matrix can't have zero rows or columns.");
        static_assert(RowCount * ColCount == sizeof...(_values),
            "Wrong number of values provided.");
        this->Set(std::forward<Values>(_values)...);
      }

      /// \brief Destructor
      public: virtual ~MatrixX() {}

      /// \brief Get the number of rows in this matrix.
      /// \return The number of rows.
      public: inline std::size_t Rows() const
      {
        return RowCount;
      }

      /// \brief Get the number of columns in this matrix.
      /// \return The number of columns.
      public: inline std::size_t Columns() const
      {
        return ColCount;
      }

      /// \brief Set new values for the matrix.
      /// \param[in] _values Values to fill matrix. The total number of passed
      /// values must be equal to RowCount * ColCount.
      public:
      template<class ... Values>
      void Set(Values... _values)
      {
        static_assert(RowCount * ColCount == sizeof...(_values),
            "Wrong number of values provided.");
#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-braces"
#endif
        this->data = {std::forward<Values>(_values)...};
#if defined(__clang__)
  #pragma clang diagnostic pop
#endif
      }

      /// \brief Change value of an element at a given row and column.
      /// \param[in] _row Row of element, must be < RowCount
      /// \param[in] _col Column of element, must be < ColCount
      /// \param[in] _value New value
      public: void SetElement(std::size_t _row, std::size_t _col,
          Precision _value)
      {
        this->Clamp(_row, _col);
        this->data.at(_row).at(_col) = _value;
      }

      /// \brief Return the transpose of this matrix
      /// \return Transpose of this matrix.
      public: MatrixX<Precision, ColCount, RowCount> Transposed() const
      {
        MatrixX<Precision, ColCount, RowCount> result;
        for (std::size_t row = 0; row < RowCount; ++row)
        {
          for (std::size_t col = 0; col < ColCount; ++col)
          {
            assert(row < RowCount);
            assert(col < ColCount);
            result.SetElement(col, row, this->data[row][col]);
          }
        }
        return result;
      }

      /// \brief Assignment operator.
      /// \param _mat Incoming matrix
      /// \return This matrix
      public: MatrixX<Precision, RowCount, ColCount> &operator=(
          const MatrixX<Precision, RowCount, ColCount> &_mat)
      {
        this->data = _mat.data;
        return *this;
      }

      /// \brief Addition operator.
      /// \param[in] _m2 Matrix to sum with.
      /// \return A new matrix with the sum.
      public: MatrixX<Precision, RowCount, ColCount> operator+(
          const MatrixX<Precision, RowCount, ColCount> &_m2) const
      {
        MatrixX<Precision, RowCount, ColCount> result;
        for (std::size_t row = 0; row < RowCount; ++row)
        {
          for (std::size_t col = 0; col < ColCount; ++col)
          {
            result.SetElement(row, col, this->data[row][col] + _m2(row, col));
          }
        }
        return result;
      }

      /// \brief Get the value at the specified row, column index
      /// \param[in] _row the row index. Index values are clamped to a
      /// range of [0, RowCount].
      /// \param[in] _col The column index. Index values are clamped to a
      /// range of [0, ColCount].
      /// \return The value at the specified index
      public: Precision operator()(std::size_t _row, std::size_t _col) const
      {
        this->Clamp(_row, _col);
        assert(_row < RowCount);
        assert(_col < ColCount);
        return this->data.at(_row).at(_col);
      }

      /// \brief Equality test with tolerance.
      /// \param[in] _m the matrix to compare to
      /// \param[in] _tol equality tolerance.
      /// \return true if the elements of the matrices are equal within
      /// the tolerence specified by _tol.
      public: bool Equal(const MatrixX<Precision, RowCount, ColCount> &_m,
          const Precision &_tol = static_cast<Precision>(1e-6)) const
      {
        for (std::size_t row = 0; row < RowCount; ++row)
        {
          for (std::size_t col = 0; col < ColCount; ++col)
          {
            assert(row < RowCount);
            assert(col < ColCount);
            if (!equal<Precision>(this->data[row][col], _m(row, col), _tol))
              return false;
          }
        }
        return true;
      }

      /// \brief Equality operator
      /// \param[in] _m Matrix3 to test
      /// \return true if the 2 matrices are equal (using the tolerance 1e-6),
      ///  false otherwise
      public: bool operator==(const MatrixX<Precision, RowCount, ColCount> &_m)
          const
      {
        return this->Equal(_m, static_cast<Precision>(1e-6));
      }

      /// \brief Inequality test operator
      /// \param[in] _m Matrix6<T> to test
      /// \return True if not equal (using the default tolerance of 1e-6)
      public: bool operator!=(const MatrixX<Precision, RowCount, ColCount> &_m)
          const
      {
        return !(*this == _m);
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _m Matrix to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
          const MatrixX<Precision, RowCount, ColCount> &_m)
      {
        for (std::size_t row = 0; row < RowCount; ++row)
        {
          for (std::size_t col = 0; col < ColCount; ++col)
          {
             _out << precision(_m(row, col), 6);
             if (!(row == (RowCount - 1) && col == (ColCount - 1)))
               _out << " ";
          }
        }
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param _in Input stream
      /// \param _m Matrix to output
      /// \return the stream
      public: friend std::istream &operator>>(std::istream &_in,
          MatrixX<Precision, RowCount, ColCount> &_m)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        for (std::size_t row = 0; row < RowCount; ++row)
        {
          for (std::size_t col = 0; col < ColCount; ++col)
          {
            Precision temp;
            _in >> temp;
            if (!_in.fail())
            {
              _m.data[row][col] = temp;
            }
          }
        }
        return _in;
      }

      /// \brief Helper function to clamp row and column values to fit this
      /// matrix.
      /// \param[in, out] _row Row value to clamp.
      /// \param[in, out] _col Column value to clamp.
      private: void Clamp(std::size_t &_row, std::size_t &_col) const
      {
        _row = clamp(_row, IGN_ZERO_SIZE_T, RowCount-1);
        _col = clamp(_col, IGN_ZERO_SIZE_T, ColCount-1);;
        assert(_row < RowCount);
        assert(_col < ColCount);
      }

      /// \brief The matrix
      private: std::array<std::array<Precision, ColCount>, RowCount> data =
          {{}};
   };

  template <std::size_t RowCount, std::size_t ColCount = RowCount>
  using MatrixXi = MatrixX<int, RowCount, ColCount>;

  template <std::size_t RowCount, std::size_t ColCount = RowCount>
  using MatrixXd = MatrixX<double, RowCount, ColCount>;

  template <std::size_t RowCount, std::size_t ColCount = RowCount>
  using MatrixXf = MatrixX<float, RowCount, ColCount>;
  }
  }
}
#endif
