//
//  vector2i.h
//  Kraken
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


#ifndef KRAKEN_VECTOR2I_H
#define KRAKEN_VECTOR2I_H

#include <functional> // for hash<>
#include <limits> // for std::numeric_limits<>
#include <math.h> // for sqrtf

namespace kraken {

class Vector2i {

public:
  union {
    struct {
      int x, y;
    };
    int c[2];
  };

  void init();
  void init(int X, int Y);
  void init(int v);
  void init(int *v);
  void init(const Vector2i &v);
  static Vector2i Create();
  static Vector2i Create(int X, int Y);
  static Vector2i Create(int v);
  static Vector2i Create(int *v);
  static Vector2i Create(const Vector2i &v);

  // Vector2 swizzle getters
  Vector2i yx() const;
    
  // Vector2 swizzle setters
  void yx(const Vector2i &v);
    
  Vector2i operator +(const Vector2i& b) const;
  Vector2i operator -(const Vector2i& b) const;
  Vector2i operator +() const;
  Vector2i operator -() const;
  Vector2i operator *(const int v) const;
  Vector2i operator /(const int v) const;
    
  Vector2i& operator +=(const Vector2i& b);
  Vector2i& operator -=(const Vector2i& b);
  Vector2i& operator *=(const int v);
  Vector2i& operator /=(const int v);
    
  // Comparison operators are implemented to allow insertion into sorted containers such as std::set
  bool operator >(const Vector2i& b) const;
  bool operator <(const Vector2i& b) const;
    
  bool operator ==(const Vector2i& b) const;
  bool operator !=(const Vector2i& b) const;
    
  int& operator[](unsigned i);
  int operator[](unsigned i) const;
    
  int sqrMagnitude() const;
  int magnitude() const;

  void normalize();
  static Vector2i Normalize(const Vector2i &v);

  static int Cross(const Vector2i &v1, const Vector2i &v2);
  static int Dot(const Vector2i &v1, const Vector2i &v2);
  static Vector2i Min(const Vector2i &v1, const Vector2i &v2);
  static Vector2i Max(const Vector2i &v1, const Vector2i &v2);

  static Vector2i Min();
  static Vector2i Max();
  static Vector2i Zero();
  static Vector2i One();
}; // class Vector2i
static_assert(std::is_pod<Vector2i>::value, "kraken::Vector2i must be a POD type.");

} // namespace kraken

namespace std {
  template<>
  struct hash<kraken::Vector2i> {
  public:
    size_t operator()(const kraken::Vector2i &s) const
    {
      size_t h1 = hash<int>()(s.x);
      size_t h2 = hash<int>()(s.y);
      return h1 ^ (h2 << 1);
    }
  };
} // namespace std

#endif // KRAKEN_VECTOR2I_H
