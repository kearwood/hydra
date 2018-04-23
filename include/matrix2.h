//
//  Matrix2.h
//  Kraken Engine / Hydra
//
//  Copyright 2018 Kearwood Gilbert. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification, are
//  permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice, this list of
//  conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice, this list
//  of conditions and the following disclaimer in the documentation and/or other materials
//  provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY KEARWOOD GILBERT ''AS IS'' AND ANY EXPRESS OR IMPLIED
//  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
//  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL KEARWOOD GILBERT OR
//  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
//  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//  The views and conclusions contained in the software and documentation are those of the
//  authors and should not be interpreted as representing official policies, either expressed
//  or implied, of Kearwood Gilbert.
//

#include "vector2.h"

#ifndef KRAKEN_MATRIX2_H
#define KRAKEN_MATRIX2_H

namespace kraken {

class Matrix2 {
public:

  union {
    struct {
      Vector2 axis_x, axis_y;
    };
    // Matrix components, in column-major order
    float c[4];
  };
    
  // Default initializer - Creates an identity matrix
  void init();
    
  void init(float *pMat);
    
  void init(const Vector2 &new_axis_x, const Vector2 &new_axis_y);

  void init(const Matrix2 &m);
    
  // Overload comparison operator
  bool operator==(const Matrix2 &m) const;
    
  // Overload compound multiply operator
  Matrix2& operator*=(const Matrix2 &m);
    
  float& operator[](unsigned i);
  float operator[](unsigned i) const;
    
  // Overload multiply operator
  //Matrix4& operator*(const Matrix4 &m);
  Matrix2 operator*(const Matrix2 &m) const;
    
  float *getPointer();
    
  void scale(float x, float y);
  void scale(const Vector2 &v);
  void scale(float s);
  void rotate(float angle);
  bool invert();
  void transpose();
    
  static Matrix2 Invert(const Matrix2 &m);
  static Matrix2 Transpose(const Matrix2 &m);
  static Vector2 Dot(const Matrix2 &m, const Vector2 &v);

  static Matrix2 Rotation(float);
  static Matrix2 Scaling(const Vector2 &v);
  static Matrix2 Identity();
};
static_assert(std::is_pod<Matrix2>::value, "kraken::Matrix2 must be a POD type.");

} // namespace kraken

namespace std {
  template<>
  struct hash<kraken::Matrix2> {
  public:
    size_t operator()(const kraken::Matrix2 &s) const
    {
      size_t h1 = hash<kraken::Vector2>()(s.axis_x);
      size_t h2 = hash<kraken::Vector2>()(s.axis_y);
      return h1 ^ (h2 << 1);
    }
  };
} // namespace std

#endif // KRAKEN_MATRIX2_H
