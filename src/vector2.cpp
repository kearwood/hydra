//
//  Vector2.cpp
//  Kraken Engine / Hydra
//
//  Copyright 2023 Kearwood Gilbert. All rights reserved.
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

#include "../include/hydra.h"
#include "krhelpers.h"

namespace hydra {

void Vector2::init()
{
  x = 0.0;
  y = 0.0;
}

Vector2 Vector2::Create()
{
  Vector2 r;
  r.init();
  return r;
}

void Vector2::init(float X, float Y)
{
  x = X;
  y = Y;
}

Vector2 Vector2::Create(float X, float Y)
{
  Vector2 r;
  r.init(X, Y);
  return r;
}

void Vector2::init(float v)
{
  x = v;
  y = v;
}

Vector2 Vector2::Create(float v)
{
  Vector2 r;
  r.init(v);
  return r;
}

void Vector2::init(float* v)
{
  x = v[0];
  y = v[1];
}

Vector2 Vector2::Create(float* v)
{
  Vector2 r;
  r.init(v);
  return r;
}

void Vector2::init(const Vector2& v)
{
  x = v.x;
  y = v.y;
}

Vector2 Vector2::Create(const Vector2& v)
{
  Vector2 r;
  r.init(v);
  return r;
}

// Vector2 swizzle getters
Vector2 Vector2::yx() const
{
  return Vector2::Create(y, x);
}

// Vector2 swizzle setters
void Vector2::yx(const Vector2& v)
{
  y = v.x;
  x = v.y;
}

Vector2 Vector2::Min()
{
  return Vector2::Create(-std::numeric_limits<float>::max());
}

Vector2 Vector2::Max()
{
  return Vector2::Create(std::numeric_limits<float>::max());
}

Vector2 Vector2::Zero()
{
  return Vector2::Create(0.0f);
}

Vector2 Vector2::One()
{
  return Vector2::Create(1.0f);
}

Vector2 Vector2::operator +(const Vector2& b) const
{
  return Vector2::Create(x + b.x, y + b.y);
}

Vector2 Vector2::operator -(const Vector2& b) const
{
  return Vector2::Create(x - b.x, y - b.y);
}

Vector2 Vector2::operator +() const
{
  return *this;
}

Vector2 Vector2::operator -() const
{
  return Vector2::Create(-x, -y);
}

Vector2 Vector2::operator *(const float v) const
{
  return Vector2::Create(x * v, y * v);
}

Vector2 Vector2::operator /(const float v) const
{
  float inv_v = 1.0f / v;
  return Vector2::Create(x * inv_v, y * inv_v);
}

Vector2& Vector2::operator +=(const Vector2& b)
{
  x += b.x;
  y += b.y;
  return *this;
}

Vector2& Vector2::operator -=(const Vector2& b)
{
  x -= b.x;
  y -= b.y;
  return *this;
}



Vector2& Vector2::operator *=(const float v)
{
  x *= v;
  y *= v;
  return *this;
}

Vector2& Vector2::operator /=(const float v)
{
  float inv_v = 1.0f / v;
  x *= inv_v;
  y *= inv_v;
  return *this;
}


bool Vector2::operator ==(const Vector2& b) const
{
  return x == b.x && y == b.y;
}

bool Vector2::operator !=(const Vector2& b) const
{
  return x != b.x || y != b.y;
}

bool Vector2::operator >(const Vector2& b) const
{
  // Comparison operators are implemented to allow insertion into sorted containers such as std::set
  if (x > b.x) {
    return true;
  } else if (x < b.x) {
    return false;
  } else if (y > b.y) {
    return true;
  } else {
    return false;
  }
}

bool Vector2::operator <(const Vector2& b) const
{
  // Comparison operators are implemented to allow insertion into sorted containers such as std::set
  if (x < b.x) {
    return true;
  } else if (x > b.x) {
    return false;
  } else if (y < b.y) {
    return true;
  } else {
    return false;
  }
}

float& Vector2::operator[] (unsigned i)
{
  switch (i) {
  case 0:
    return x;
  case 1:
  default:
    return y;
  }
}

float Vector2::operator[](unsigned i) const
{
  switch (i) {
  case 0:
    return x;
  case 1:
  default:
    return y;
  }
}

void Vector2::normalize()
{
  float inv_magnitude = 1.0f / sqrtf(x * x + y * y);
  x *= inv_magnitude;
  y *= inv_magnitude;
}

float Vector2::sqrMagnitude() const
{
  return x * x + y * y;
}

float Vector2::magnitude() const
{
  return sqrtf(x * x + y * y);
}


Vector2 Vector2::Normalize(const Vector2& v)
{
  float inv_magnitude = 1.0f / sqrtf(v.x * v.x + v.y * v.y);
  return Vector2::Create(v.x * inv_magnitude, v.y * inv_magnitude);
}

float Vector2::Cross(const Vector2& v1, const Vector2& v2)
{
  return v1.x * v2.y - v1.y * v2.x;
}

float Vector2::Dot(const Vector2& v1, const Vector2& v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

Vector2 Vector2::Min(const Vector2& v1, const Vector2& v2)
{
  return Vector2::Create(KRMIN(v1.x, v2.x), KRMIN(v1.y, v2.y));
}

Vector2 Vector2::Max(const Vector2& v1, const Vector2& v2)
{
  return Vector2::Create(KRMAX(v1.x, v2.x), KRMAX(v1.y, v2.y));
}

} // namepsace hydra
