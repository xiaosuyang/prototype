/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*! @file MathUtilities.h
 *  @brief Utility functions for math
 *
 */

#ifndef PROJECT_MATHUTILITIES_H
#define PROJECT_MATHUTILITIES_H

#include <Eigen/Dense>
#include <cmath>



class LowPassFilter
{
public:
  LowPassFilter() {};

  // 更新滤波器输出
  double update(double input)
  {
    double output = alpha_ * input + (1.0 - alpha_) * prev_output_;
    prev_output_ = output;
    return output;
  }
  void set(double sample_rate, double cutoff_frequency)
  {
    double dt = 1.0 / sample_rate;
    double RC = 1.0 / (cutoff_frequency * 2.0 * M_PI);
    alpha_ = dt / (dt + RC);
    prev_output_ = 0.0;
  }

private:
  double alpha_;
  double prev_output_;
};

/*!
 * Square a number
 */
template <typename T>
T square(T a)
{
  return a * a;
}

/*!
 * Are two eigen matrices almost equal?
 */
template <typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T> &a, const Eigen::MatrixBase<T> &b,
                 T2 tol)
{
  long x = T::RowsAtCompileTime;
  long y = T::ColsAtCompileTime;

  if (T::RowsAtCompileTime == Eigen::Dynamic ||
      T::ColsAtCompileTime == Eigen::Dynamic)
  {
    assert(a.rows() == b.rows());
    assert(a.cols() == b.cols());
    x = a.rows();
    y = a.cols();
  }

  for (long i = 0; i < x; i++)
  {
    for (long j = 0; j < y; j++)
    {
      T2 error = std::abs(a(i, j) - b(i, j));
      if (error >= tol)
        return false;
    }
  }
  return true;
}

template <typename T>
float rad2deg(T rad)
{
  return rad / M_PI * 180;
}

template <typename T>
float deg2rad(T deg)
{
  return deg / 180 * M_PI;
}

template <typename T>
float numderivative(T f2, T f1, float stime)
{
  return (f2 - f1) / (2 * stime);
}

template <typename T>
float secondaryderivative(T f3, T f2, T f1, float stime)
{
  stime = stime / 2;
  float V1 = numderivative(f2, f1, stime);
  float V2 = numderivative(f3, f2, stime);

  return (V2 - V1) / (2 * stime);
}

template <typename T>
float sign(T num)
{
  if (num > 0)
    return 1;
  else if (num < 0)
    return -1;
  else
    return 0;
}

struct Point
{
  double x;
  double y;
};

Eigen::MatrixXf Matrix_Pow(Eigen::MatrixXf Mat, size_t n);


#endif // PROJECT_MATHUTILITIES_H
