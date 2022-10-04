//
//  Vector3i.cpp
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

#include "../include/hydra.h"
#include "krhelpers.h"

namespace kraken {

//default constructor
void Vector3i::init()
{
  x = 0;
  y = 0;
  z = 0;
}

Vector3i Vector3i::Create()
{
  Vector3i r;
  r.init();
  return r;
}

void Vector3i::init(const Vector3i& v)
{
  x = v.x;
  y = v.y;
  z = v.z;
}

Vector3i Vector3i::Create(const Vector3i& v)
{
  Vector3i r;
  r.init(v);
  return r;
}

void Vector3i::init(int* v)
{
  x = v[0];
  y = v[1];
  z = v[2];
}

Vector3i Vector3i::Create(int* v)
{
  Vector3i r;
  r.init(v);
  return r;
}

void Vector3i::init(int v)
{
  x = v;
  y = v;
  z = v;
}

Vector3i Vector3i::Create(int v)
{
  Vector3i r;
  r.init(v);
  return r;
}

void Vector3i::init(int X, int Y, int Z)
{
  x = X;
  y = Y;
  z = Z;
}

Vector3i Vector3i::Create(int X, int Y, int Z)
{
  Vector3i r;
  r.init(X, Y, Z);
  return r;
}

Vector2i Vector3i::xx() const
{
  return Vector2i::Create(x, x);
}

Vector2i Vector3i::xy() const
{
  return Vector2i::Create(x, y);
}

Vector2i Vector3i::xz() const
{
  return Vector2i::Create(x, z);
}

Vector2i Vector3i::yx() const
{
  return Vector2i::Create(y, x);
}

Vector2i Vector3i::yy() const
{
  return Vector2i::Create(y, y);
}

Vector2i Vector3i::yz() const
{
  return Vector2i::Create(y, z);
}

Vector2i Vector3i::zx() const
{
  return Vector2i::Create(z, x);
}

Vector2i Vector3i::zy() const
{
  return Vector2i::Create(z, y);
}

Vector2i Vector3i::zz() const
{
  return Vector2i::Create(z, z);
}

void Vector3i::xy(const Vector2i& v)
{
  x = v.x;
  y = v.y;
}

void Vector3i::xz(const Vector2i& v)
{
  x = v.x;
  z = v.y;
}

void Vector3i::yx(const Vector2i& v)
{
  y = v.x;
  x = v.y;
}

void Vector3i::yz(const Vector2i& v)
{
  y = v.x;
  z = v.y;
}

void Vector3i::zx(const Vector2i& v)
{
  z = v.x;
  x = v.y;
}

void Vector3i::zy(const Vector2i& v)
{
  z = v.x;
  y = v.y;
}

Vector3i Vector3i::Min()
{
  return Vector3i::Create(-std::numeric_limits<int>::max());
}

Vector3i Vector3i::Max()
{
  return Vector3i::Create(std::numeric_limits<int>::max());
}

Vector3i Vector3i::Zero()
{
  return Vector3i::Create();
}

Vector3i Vector3i::One()
{
  return Vector3i::Create(1,1,1);
}

Vector3i Vector3i::Forward()
{
  return Vector3i::Create(0, 0, 1);
}

Vector3i Vector3i::Backward()
{
  return Vector3i::Create(0, 0, -1);
}

Vector3i Vector3i::Up()
{
  return Vector3i::Create(0,1,0);
}

Vector3i Vector3i::Down()
{
  return Vector3i::Create(0, -1, 0);
}

Vector3i Vector3i::Left()
{
  return Vector3i::Create(-1, 0, 0);
}

Vector3i Vector3i::Right()
{
  return Vector3i::Create(1,0,0);
}


void Vector3i::scale(const Vector3i& v)
{
  x *= v.x;
  y *= v.y;
  z *= v.z;
}

Vector3i Vector3i::Scale(const Vector3i& v1, const Vector3i& v2)
{
  return Vector3i::Create(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
}

Vector3i Vector3i::operator +(const Vector3i& b) const
{
  return Vector3i::Create(x + b.x, y + b.y, z + b.z);
}
Vector3i Vector3i::operator -(const Vector3i& b) const
{
  return Vector3i::Create(x - b.x, y - b.y, z - b.z);
}
Vector3i Vector3i::operator +() const
{
  return *this;
}
Vector3i Vector3i::operator -() const
{
  return Vector3i::Create(-x, -y, -z);
}

Vector3i Vector3i::operator *(const int v) const
{
  return Vector3i::Create(x * v, y * v, z * v);
}

Vector3i Vector3i::operator /(const int v) const
{
  return Vector3i::Create(x / v, y / v, z / v);
}

Vector3i& Vector3i::operator +=(const Vector3i& b)
{
  x += b.x;
  y += b.y;
  z += b.z;

  return *this;
}

Vector3i& Vector3i::operator -=(const Vector3i& b)
{
  x -= b.x;
  y -= b.y;
  z -= b.z;

  return *this;
}

Vector3i& Vector3i::operator *=(const int v)
{
  x *= v;
  y *= v;
  z *= v;

  return *this;
}

Vector3i& Vector3i::operator /=(const int v)
{
  x /= v;
  y /= v;
  z /= v;

  return *this;
}

bool Vector3i::operator ==(const Vector3i& b) const
{
  return x == b.x && y == b.y && z == b.z;

}
bool Vector3i::operator !=(const Vector3i& b) const
{
  return x != b.x || y != b.y || z != b.z;
}

int& Vector3i::operator[](unsigned i)
{
  switch (i) {
  case 0:
    return x;
  case 1:
    return y;
  default:
  case 2:
    return z;
  }
}

int Vector3i::operator[](unsigned i) const
{
  switch (i) {
  case 0:
    return x;
  case 1:
    return y;
  case 2:
  default:
    return z;
  }
}

int Vector3i::sqrMagnitude() const
{
  // calculate the square of the magnitude (useful for comparison of magnitudes without the cost of a sqrt() function)
  return x * x + y * y + z * z;
}

int Vector3i::magnitude() const
{
  return sqrt(sqrMagnitude());
}

Vector3i Vector3i::Cross(const Vector3i& v1, const Vector3i& v2)
{
  return Vector3i::Create(v1.y * v2.z - v1.z * v2.y,
                          v1.z * v2.x - v1.x * v2.z,
                          v1.x * v2.y - v1.y * v2.x);
}

int Vector3i::Dot(const Vector3i& v1, const Vector3i& v2)
{
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3i Vector3i::Min(const Vector3i& v1, const Vector3i& v2)
{
  return Vector3i::Create(KRMIN(v1.x, v2.x), KRMIN(v1.y, v2.y), KRMIN(v1.z, v2.z));
}

Vector3i Vector3i::Max(const Vector3i& v1, const Vector3i& v2)
{
  return Vector3i::Create(KRMAX(v1.x, v2.x), KRMAX(v1.y, v2.y), KRMAX(v1.z, v2.z));
}

bool Vector3i::operator >(const Vector3i& b) const
{
  // Comparison operators are implemented to allow insertion into sorted containers such as std::set
  if (x > b.x) {
    return true;
  } else if (x < b.x) {
    return false;
  } else if (y > b.y) {
    return true;
  } else if (y < b.y) {
    return false;
  } else if (z > b.z) {
    return true;
  } else {
    return false;
  }
}

bool Vector3i::operator <(const Vector3i& b) const
{
  // Comparison operators are implemented to allow insertion into sorted containers such as std::set
  if (x < b.x) {
    return true;
  } else if (x > b.x) {
    return false;
  } else if (y < b.y) {
    return true;
  } else if (y > b.y) {
    return false;
  } else if (z < b.z) {
    return true;
  } else {
    return false;
  }
}

} // namespace kraken
