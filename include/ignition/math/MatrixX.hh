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
    /// \brief A matrix with any given size.
    template<typename Precision, std::size_t Rows, std::size_t Columns>
    class MatrixX
    {
      /// \brief Constructor
      public: MatrixX()
      {
        static_assert(Rows > 0 && Columns > 0,
            "Matrix can't have zero rows or columns.");

        for (std::size_t row = 0; row < Rows; ++row)
        {
          for (std::size_t col = 0; col < Columns; ++col)
          {
            this->data[row][col] = static_cast<Precision>(0);
          }
        }
      }

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      public: MatrixX(const MatrixX<Precision, Rows, Columns> &_m)
      {
        this->data = _m.data;
      }

      /// \brief Move constructor
      /// \param _m Matrix to move
      public: MatrixX(const MatrixX<Precision, Rows, Columns> &&_m)
      {
        this->data = std::move(_m.data);
      }

      /// \brief Constructor
      /// \param[in] _values Values to fill matrix
      public:
      template<class ... Values>
      MatrixX(Values... _values)
      {
        static_assert(Rows > 0 && Columns > 0,
            "Matrix can't have zero rows or columns.");
        static_assert(Rows * Columns == sizeof...(_values),
            "Wrong number of values provided.");
        this->Set(std::forward<Values>(_values)...);
      }

      /// \brief Destructor
      public: virtual ~MatrixX() {};

      /// \brief Change the values
      /// \param[in] _values Values to set.
      public:
      template<class ... Values>
      void Set(Values... _values)
      {
        static_assert(Rows * Columns == sizeof...(_values),
            "Wrong number of values provided.");
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-braces"
        this->data = {std::forward<Values>(_values)...};
#pragma clang diagnostic pop
      }

      /// \brief Change value at an index.
      /// \param[in] _row
      /// \param[in] _col
      /// \param[in] _value
      public: void SetElement(std::size_t _row, std::size_t _col,
          Precision _value)
      {
        this->Clamp(_row, _col, _row, _col);
        this->data.at(_row).at(_col) = _value;
      }

      /// \brief Return the transpose of this matrix
      /// \return Transpose of this matrix.
      public: MatrixX<Precision, Columns, Rows> Transposed() const
      {
        MatrixX<Precision, Columns, Rows> result;
        for (std::size_t row = 0; row < Rows; ++row)
        {
          for (std::size_t col = 0; col < Columns; ++col)
          {
            assert(row < Rows);
            assert(col < Columns);
            result.SetElement(col, row, this->data[row][col]);
          }
        }
        return result;
      }

      /// \brief Equal operator. this = _mat
      /// \param _mat Incoming matrix
      /// \return itself
      public: MatrixX<Precision, Rows, Columns> &operator=(
          const MatrixX<Precision, Rows, Columns> &_mat)
      {
        this->data = _mat.data;
        return *this;
      }

      /// \brief Addition operator.
      /// \param[in] _m2 Matrix to sum with.
      /// \return A new matrix with the sum.
      public: MatrixX<Precision, Rows, Columns> operator+(
          const MatrixX<Precision, Rows, Columns> &_m2) const
      {
        MatrixX<Precision, Rows, Columns> result;
        for (std::size_t row = 0; row < Rows; ++row)
        {
          for (std::size_t col = 0; col < Columns; ++col)
          {
            result.SetElement(row, col, this->data[row][col] + _m2(row, col));
          }
        }
        return result;
      }

      /// \brief Get the value at the specified row, column index
      /// \param[in] _row the row index. Index values are clamped to a
      /// range of [0, Rows].
      /// \param[in] _col The column index. Index values are clamped to a
      /// range of [0, Columns].
      /// \return The value at the specified index
      public: Precision operator()(std::size_t _row, std::size_t _col) const
      {
        this->Clamp(_row, _col, _row, _col);
        assert(_row < Rows);
        assert(_col < Columns);
        return this->data.at(_row).at(_col);
      }

      /// \brief Equality test with tolerance.
      /// \param[in] _m the matrix to compare to
      /// \param[in] _tol equality tolerance.
      /// \return true if the elements of the matrices are equal within
      /// the tolerence specified by _tol.
      public: bool Equal(const MatrixX<Precision, Rows, Columns> &_m,
          const Precision &_tol = static_cast<Precision>(1e-6)) const
      {
        for (std::size_t row = 0; row < Rows; ++row)
        {
          for (std::size_t col = 0; col < Columns; ++col)
          {
            assert(row < Rows);
            assert(col < Columns);
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
      public: bool operator==(const MatrixX<Precision, Rows, Columns> &_m) const
      {
        return this->Equal(_m, static_cast<Precision>(1e-6));
      }

      /// \brief Inequality test operator
      /// \param[in] _m Matrix6<T> to test
      /// \return True if not equal (using the default tolerance of 1e-6)
      public: bool operator!=(const MatrixX<Precision, Rows, Columns> &_m) const
      {
        return !(*this == _m);
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _m Matrix to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
          const MatrixX<Precision, Rows, Columns> &_m)
      {
        for (std::size_t row = 0; row < Rows; ++row)
        {
          for (std::size_t col = 0; col < Columns; ++col)
          {
             _out << precision(_m(row, col), 6);
             if (!(row == (Rows - 1) && col == (Columns - 1)))
               _out << " ";
          }
        }
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param _out output stream
      /// \param _m Matrix to output
      /// \return the stream
      public: friend std::istream &operator>>(std::istream &_in,
          MatrixX<Precision, Rows, Columns> &_m)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        for (std::size_t row = 0; row < Rows; ++row)
        {
          for (std::size_t col = 0; col < Columns; ++col)
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

      private: void Clamp(std::size_t _inRow, std::size_t _inCol,
          std::size_t &_outRow, std::size_t &_outCol) const
      {
        _outRow = clamp(_inRow, IGN_ZERO_SIZE_T, Rows-1);
        _outCol = clamp(_inCol, IGN_ZERO_SIZE_T, Columns-1);;
        assert(_outRow < Rows);
        assert(_outCol < Columns);
      }

      /// \brief The matrix
      private: std::array<std::array<Precision, Columns>, Rows> data = {{}};
   };

  template <std::size_t Rows, std::size_t Columns>
  using MatrixXi = MatrixX<int, Rows, Columns>;

  template <std::size_t Rows, std::size_t Columns>
  using MatrixXd = MatrixX<double, Rows, Columns>;

  template <std::size_t Rows, std::size_t Columns>
  using MatrixXf = MatrixX<float, Rows, Columns>;
  }
  }
}
#endif
