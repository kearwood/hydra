//
//  quaternion.cpp
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

void Quaternion::init()
{
  c[0] = 1.0;
  c[1] = 0.0;
  c[2] = 0.0;
  c[3] = 0.0;
}

Quaternion Quaternion::Create()
{
  Quaternion r;
  r.init();
  return r;
}

void Quaternion::init(float w, float x, float y, float z)
{
  c[0] = w;
  c[1] = x;
  c[2] = y;
  c[3] = z;
}

Quaternion Quaternion::Create(float w, float x, float y, float z)
{
  Quaternion r;
  r.init(w, x, y, z);
  return r;
}

void Quaternion::init(const Quaternion& p)
{
  c[0] = p[0];
  c[1] = p[1];
  c[2] = p[2];
  c[3] = p[3];
}

Quaternion Quaternion::Create(const Quaternion& p)
{
  Quaternion r;
  r.init(p);
  return r;
}

void Quaternion::init(const Vector3& euler)
{
  setEulerZYX(euler);
}

Quaternion Quaternion::Create(const Vector3& euler)
{
  Quaternion r;
  r.init(euler);
  return r;
}

void Quaternion::init(const Vector3& from_vector, const Vector3& to_vector)
{

  Vector3 a = Vector3::Cross(from_vector, to_vector);
  c[0] = a[0];
  c[1] = a[1];
  c[2] = a[2];
  c[3] = sqrtf(from_vector.sqrMagnitude() * to_vector.sqrMagnitude()) + Vector3::Dot(from_vector, to_vector);
  normalize();
}

Quaternion Quaternion::Create(const Vector3& from_vector, const Vector3& to_vector)
{
  Quaternion r;
  r.init(from_vector, to_vector);
  return r;
}

void Quaternion::setEulerXYZ(const Vector3& euler)
{
  *this = Quaternion::FromAngleAxis(Vector3::Create(1.0f, 0.0f, 0.0f), euler.x)
    * Quaternion::FromAngleAxis(Vector3::Create(0.0f, 1.0f, 0.0f), euler.y)
    * Quaternion::FromAngleAxis(Vector3::Create(0.0f, 0.0f, 1.0f), euler.z);
}

void Quaternion::setEulerZYX(const Vector3& euler)
{
  // ZYX Order!
  float c1 = cosf(euler[0] * 0.5f);
  float c2 = cosf(euler[1] * 0.5f);
  float c3 = cosf(euler[2] * 0.5f);
  float s1 = sinf(euler[0] * 0.5f);
  float s2 = sinf(euler[1] * 0.5f);
  float s3 = sinf(euler[2] * 0.5f);

  c[0] = c1 * c2 * c3 + s1 * s2 * s3;
  c[1] = s1 * c2 * c3 - c1 * s2 * s3;
  c[2] = c1 * s2 * c3 + s1 * c2 * s3;
  c[3] = c1 * c2 * s3 - s1 * s2 * c3;
}

float Quaternion::operator [](unsigned i) const
{
  return c[i];
}

float& Quaternion::operator [](unsigned i)
{
  return c[i];
}

Vector3 Quaternion::eulerXYZ() const
{
  float a2 = 2 * (c[0] * c[2] - c[1] * c[3]);
  if (a2 <= -0.99999) {
    return Vector3::Create(
       2.0f * atan2f(c[1], c[0]),
       -PI * 0.5f,
       0
    );
  } else if (a2 >= 0.99999) {
    return Vector3::Create(
       2.0f * atan2f(c[1], c[0]),
       PI * 0.5f,
       0
    );
  } else {
    return Vector3::Create(
         atan2f(2 * (c[0] * c[1] + c[2] * c[3]), (1 - 2 * (c[1] * c[1] + c[2] * c[2]))),
         asinf(a2),
         atan2f(2 * (c[0] * c[3] + c[1] * c[2]), (1 - 2 * (c[2] * c[2] + c[3] * c[3])))
    );
  }


}

bool operator ==(Quaternion& v1, Quaternion& v2)
{
  return
    v1[0] == v2[0]
    && v1[1] == v2[1]
    && v1[2] == v2[2]
    && v1[3] == v2[3];
}

bool operator !=(Quaternion& v1, Quaternion& v2)
{
  return
    v1[0] != v2[0]
    || v1[1] != v2[1]
    || v1[2] != v2[2]
    || v1[3] != v2[3];
}

Quaternion Quaternion::operator *(const Quaternion& v)
{
  float t0 = (c[3] - c[2]) * (v[2] - v[3]);
  float t1 = (c[0] + c[1]) * (v[0] + v[1]);
  float t2 = (c[0] - c[1]) * (v[2] + v[3]);
  float t3 = (c[3] + c[2]) * (v[0] - v[1]);
  float t4 = (c[3] - c[1]) * (v[1] - v[2]);
  float t5 = (c[3] + c[1]) * (v[1] + v[2]);
  float t6 = (c[0] + c[2]) * (v[0] - v[3]);
  float t7 = (c[0] - c[2]) * (v[0] + v[3]);
  float t8 = t5 + t6 + t7;
  float t9 = (t4 + t8) / 2;

  return Quaternion::Create(
      t0 + t9 - t5,
      t1 + t9 - t8,
      t2 + t9 - t7,
      t3 + t9 - t6
  );
}

Quaternion Quaternion::operator *(float v) const
{
  return Quaternion::Create(c[0] * v, c[1] * v, c[2] * v, c[3] * v);
}

Quaternion Quaternion::operator /(float num) const
{
  float inv_num = 1.0f / num;
  return Quaternion::Create(c[0] * inv_num, c[1] * inv_num, c[2] * inv_num, c[3] * inv_num);
}

Quaternion Quaternion::operator +(const Quaternion& v) const
{
  return Quaternion::Create(c[0] + v[0], c[1] + v[1], c[2] + v[2], c[3] + v[3]);
}

Quaternion Quaternion::operator -(const Quaternion& v) const
{
  return Quaternion::Create(c[0] - v[0], c[1] - v[1], c[2] - v[2], c[3] - v[3]);
}

Quaternion& Quaternion::operator +=(const Quaternion& v)
{
  c[0] += v[0];
  c[1] += v[1];
  c[2] += v[2];
  c[3] += v[3];
  return *this;
}

Quaternion& Quaternion::operator -=(const Quaternion& v)
{
  c[0] -= v[0];
  c[1] -= v[1];
  c[2] -= v[2];
  c[3] -= v[3];
  return *this;
}

Quaternion& Quaternion::operator *=(const Quaternion& v)
{
  float t0 = (c[3] - c[2]) * (v[2] - v[3]);
  float t1 = (c[0] + c[1]) * (v[0] + v[1]);
  float t2 = (c[0] - c[1]) * (v[2] + v[3]);
  float t3 = (c[3] + c[2]) * (v[0] - v[1]);
  float t4 = (c[3] - c[1]) * (v[1] - v[2]);
  float t5 = (c[3] + c[1]) * (v[1] + v[2]);
  float t6 = (c[0] + c[2]) * (v[0] - v[3]);
  float t7 = (c[0] - c[2]) * (v[0] + v[3]);
  float t8 = t5 + t6 + t7;
  float t9 = (t4 + t8) / 2;

  c[0] = t0 + t9 - t5;
  c[1] = t1 + t9 - t8;
  c[2] = t2 + t9 - t7;
  c[3] = t3 + t9 - t6;

  return *this;
}

Quaternion& Quaternion::operator *=(const float& v)
{
  c[0] *= v;
  c[1] *= v;
  c[2] *= v;
  c[3] *= v;
  return *this;
}

Quaternion& Quaternion::operator /=(const float& v)
{
  float inv_v = 1.0f / v;
  c[0] *= inv_v;
  c[1] *= inv_v;
  c[2] *= inv_v;
  c[3] *= inv_v;
  return *this;
}

Quaternion Quaternion::operator +() const
{
  return *this;
}

Quaternion Quaternion::operator -() const
{
  return Quaternion::Create(-c[0], -c[1], -c[2], -c[3]);
}

Quaternion Quaternion::Normalize(const Quaternion& v1)
{
  float inv_magnitude = 1.0f / sqrtf(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + v1[3] * v1[3]);
  return Quaternion::Create(
      v1[0] * inv_magnitude,
      v1[1] * inv_magnitude,
      v1[2] * inv_magnitude,
      v1[3] * inv_magnitude
  );
}

void Quaternion::normalize()
{
  float inv_magnitude = 1.0f / sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2] + c[3] * c[3]);
  c[0] *= inv_magnitude;
  c[1] *= inv_magnitude;
  c[2] *= inv_magnitude;
  c[3] *= inv_magnitude;
}

Quaternion Quaternion::Conjugate(const Quaternion& v1)
{
  return Quaternion::Create(v1[0], -v1[1], -v1[2], -v1[3]);
}

void Quaternion::conjugate()
{
  c[1] = -c[1];
  c[2] = -c[2];
  c[3] = -c[3];
}

void Quaternion::invert()
{
  conjugate();
  normalize();
}

Quaternion Quaternion::Invert(const Quaternion& v1)
{
  return Normalize(Conjugate(v1));
}

Matrix4 Quaternion::rotationMatrix() const
{
  Matrix4 matRotate;
  matRotate.init();

  /*
  Vector3 euler = eulerXYZ();

  matRotate.rotate(euler.x, X_AXIS);
  matRotate.rotate(euler.y, Y_AXIS);
  matRotate.rotate(euler.z, Z_AXIS);
   */

   // FINDME - Determine why the more optimal routine commented below wasn't working


  matRotate.c[0] = 1.0f - 2.0f * (c[2] * c[2] + c[3] * c[3]);
  matRotate.c[1] = 2.0f * (c[1] * c[2] - c[0] * c[3]);
  matRotate.c[2] = 2.0f * (c[0] * c[2] + c[1] * c[3]);

  matRotate.c[4] = 2.0f * (c[1] * c[2] + c[0] * c[3]);
  matRotate.c[5] = 1.0f - 2.0f * (c[1] * c[1] + c[3] * c[3]);
  matRotate.c[6] = 2.0f * (c[2] * c[3] - c[0] * c[1]);

  matRotate.c[8] = 2.0f * (c[1] * c[3] - c[0] * c[2]);
  matRotate.c[9] = 2.0f * (c[0] * c[1] + c[2] * c[3]);
  matRotate.c[10] = 1.0f - 2.0f * (c[1] * c[1] + c[2] * c[2]);

  return matRotate;
}


Quaternion Quaternion::FromAngleAxis(const Vector3& axis, float angle)
{
  float ha = angle * 0.5f;
  float sha = sinf(ha);
  return Quaternion::Create(cosf(ha), axis.x * sha, axis.y * sha, axis.z * sha);
}

Quaternion Quaternion::FromRotationMatrix(const Matrix4& m)
{
  // see http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
  const float trace = m[0] + m[5] + m[10];
  float w, x, y, z;
  if (trace > 0.0) {
    const float s = 0.5f / sqrtf(trace + 1.0f);
    w = 0.25f / s;
    x = (m[9] - m[6]) * s;
    y = (m[2] - m[8]) * s;
    z = (m[4] - m[1]) * s;
  } else if (m[0] > m[5] && m[0] > m[10]) {
    const float s = 2.0f * sqrtf(1.0f + m[0] - m[5] - m[10]);
    w = (m[9] - m[6]) / s;
    x = 0.25f * s;
    y = (m[1] + m[4]) / s;
    z = (m[2] + m[8]) / s;
  } else if (m[5] > m[10]) {
    const float s = 2.0f * sqrtf(1.0f + m[5] - m[0] - m[10]);
    w = (m[2] - m[8]) / s;
    x = (m[1] + m[4]) / s;
    y = 0.25f * s;
    z = (m[6] + m[9]) / s;
  } else {
    const float s = 2.0f * sqrtf(1.0f + m[10] - m[0] - m[5]);
    w = (m[4] - m[1]) / s;
    x = (m[2] + m[8]) / s;
    y = (m[6] + m[9]) / s;
    z = 0.25f * s;
  }
  return Quaternion::Create(w, x, y, z);
}

float Quaternion::Dot(const Quaternion& v1, const Quaternion& v2)
{
  return v1.c[0] * v2.c[0] + v1.c[1] * v2.c[1] + v1.c[2] * v2.c[2] + v1.c[3] * v2.c[3];
}

Quaternion Quaternion::Lerp(const Quaternion& a, const Quaternion& b, float t)
{
  if (t <= 0.0f) {
    return a;
  } else if (t >= 1.0f) {
    return b;
  }

  return a * (1.0f - t) + b * t;
}

Quaternion Quaternion::Slerp(const Quaternion& a, const Quaternion& b, float t)
{
  if (t <= 0.0f) {
    return a;
  }

  if (t >= 1.0f) {
    return b;
  }

  float coshalftheta = Dot(a, b);
  Quaternion c = a;

  // Angle is greater than 180. We can negate the angle/quat to get the
  // shorter rotation to reach the same destination.
  if (coshalftheta < 0.0f) {
    coshalftheta = -coshalftheta;
    c = -c;
  }

  if (coshalftheta > (1.0f - std::numeric_limits<float>::epsilon())) {
    // Angle is tiny - save some computation by lerping instead.
    return Lerp(c, b, t);
  }

  float halftheta = acosf(coshalftheta);

  return (c * sinf((1.0f - t) * halftheta) + b * sinf(t * halftheta)) / sinf(halftheta);
}

} // namespace hydra
