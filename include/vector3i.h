//
//  Vector3i.h
//  Kraken Engine / Hydra
//
//  Copyright 2022 Kearwood Gilbert. All rights reserved.
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

#pragma once

#include <functional> // for hash<>

#include "vector2i.h"

namespace kraken {

class Vector3i
{

public:
  union
  {
    struct
    {
      int x, y, z;
    };
    int c[3];
  };

  void init();
  void init(int X, int Y, int Z);
  void init(int v);
  void init(int* v);
  void init(const Vector3i& v);
  static Vector3i Create();
  static Vector3i Create(int X, int Y, int Z);
  static Vector3i Create(int v);
  static Vector3i Create(int* v);
  static Vector3i Create(const Vector3i& v);


  // Vector2 swizzle getters
  Vector2i xx() const;
  Vector2i xy() const;
  Vector2i xz() const;
  Vector2i yx() const;
  Vector2i yy() const;
  Vector2i yz() const;
  Vector2i zx() const;
  Vector2i zy() const;
  Vector2i zz() const;

  // Vector2 swizzle setters
  void xy(const Vector2i& v);
  void xz(const Vector2i& v);
  void yx(const Vector2i& v);
  void yz(const Vector2i& v);
  void zx(const Vector2i& v);
  void zy(const Vector2i& v);

  Vector3i operator +(const Vector3i& b) const;
  Vector3i operator -(const Vector3i& b) const;
  Vector3i operator +() const;
  Vector3i operator -() const;
  Vector3i operator *(const int v) const;
  Vector3i operator /(const int v) const;

  Vector3i& operator +=(const Vector3i& b);
  Vector3i& operator -=(const Vector3i& b);
  Vector3i& operator *=(const int v);
  Vector3i& operator /=(const int v);

  bool operator ==(const Vector3i& b) const;
  bool operator !=(const Vector3i& b) const;

  // Comparison operators are implemented to allow insertion into sorted containers such as std::set
  bool operator >(const Vector3i& b) const;
  bool operator <(const Vector3i& b) const;

  int& operator[](unsigned i);
  int operator[](unsigned i) const;

  int sqrMagnitude() const; // calculate the square of the magnitude (useful for comparison of magnitudes without the cost of a sqrt() function)
  int magnitude() const;

  void scale(const Vector3i& v);

  static Vector3i Cross(const Vector3i& v1, const Vector3i& v2);
  static int Dot(const Vector3i& v1, const Vector3i& v2);
  static Vector3i Min(const Vector3i& v1, const Vector3i& v2);
  static Vector3i Max(const Vector3i& v1, const Vector3i& v2);

  static Vector3i Min();
  static Vector3i Max();
  static Vector3i Zero();
  static Vector3i One();
  static Vector3i Forward();
  static Vector3i Backward();
  static Vector3i Up();
  static Vector3i Down();
  static Vector3i Left();
  static Vector3i Right();
  static Vector3i Scale(const Vector3i& v1, const Vector3i& v2);
};
static_assert(std::is_pod<Vector3i>::value, "kraken::Vector3i must be a POD type.");

} // namespace kraken

namespace std {
template<>
struct hash<kraken::Vector3i>
{
public:
  size_t operator()(const kraken::Vector3i& s) const
  {
    size_t h1 = hash<int>()(s.x);
    size_t h2 = hash<int>()(s.y);
    size_t h3 = hash<int>()(s.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};
} // namespace std
