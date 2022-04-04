//
//  matrix23.cpp
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

#include <string.h>

namespace kraken {

void Matrix2x3::init() {
    // Default constructor - Initialize with an identity matrix
    static const float IDENTITY_MATRIX[] = {
        1.0, 0.0,
        0.0, 1.0,
        0.0, 0.0
    };
    memcpy(c, IDENTITY_MATRIX, sizeof(float) * 6);
    
}

void Matrix2x3::init(float *pMat) {
    memcpy(c, pMat, sizeof(float) * 6);
}

void Matrix2x3::init(const Vector2 &new_axis_x, const Vector2 &new_axis_y, const Vector2 &new_transform)
{
    c[0]  = new_axis_x.x;    c[1]  = new_axis_x.y;
    c[2]  = new_axis_y.x;    c[3]  = new_axis_y.y;
    c[4] = new_transform.x;  c[5] = new_transform.y;
}

void Matrix2x3::init(const Matrix2x3 &m) {
    memcpy(c, m.c, sizeof(float) * 6);
}

float *Matrix2x3::getPointer() {
    return c;
}

float& Matrix2x3::operator[](unsigned i) {
    return c[i];
}

float Matrix2x3::operator[](unsigned i) const {
    return c[i];
}

// Overload comparison operator
bool Matrix2x3::operator==(const Matrix2x3 &m) const {
    return memcmp(c, m.c, sizeof(float) * 6) == 0;
}

// Overload compound multiply operator
Matrix2x3& Matrix2x3::operator*=(const Matrix2x3 &m) {
    float temp[6];
/*
    temp[0] = c[0] * m[0] + c[2] * m[1];
    temp[1] = c[1] * m[0] + c[3] * m[1];
    temp[2] = c[0] * m[2] + c[2] * m[3];
    temp[3] = c[1] * m[2] + c[3] * m[3];
    temp[4] = c[0] * m[4] + c[2] * m[5] + c[4];
    temp[5] = c[1] * m[4] + c[3] * m[5] + c[5];
*/
    temp[0] = m[0] * c[0] + m[2] * c[1];
    temp[1] = m[1] * c[0] + m[3] * c[1];
    temp[2] = m[0] * c[2] + m[2] * c[3];
    temp[3] = m[1] * c[2] + m[3] * c[3];
    temp[4] = m[0] * c[4] + m[2] * c[5] + m[4];
    temp[5] = m[1] * c[4] + m[3] * c[5] + m[5];

    memcpy(c, temp, sizeof(float) * 6);
    return *this;
}

// Overload multiply operator
Matrix2x3 Matrix2x3::operator*(const Matrix2x3 &m) const {
    Matrix2x3 ret = *this;
    ret *= m;
    return ret;
}

/* Perform translation operations on a matrix */
void Matrix2x3::translate(float x, float y) {
    Matrix2x3 newMatrix; // Create new identity matrix
    newMatrix.init();
    
    newMatrix.transform.x = x;
    newMatrix.transform.y = y;
    
    *this *= newMatrix;
}

void Matrix2x3::translate(const Vector2 &v)
{
    translate(v.x, v.y);
}

/* Rotate a matrix by an angle on a X, Y, or Z axis */
void Matrix2x3::rotate(float angle) {
    Matrix2x3 newMatrix;
    newMatrix.init();
    newMatrix.c[0] = cosf(angle);
    newMatrix.c[1] = -sinf(angle);
    newMatrix.c[2] = -newMatrix.c[1];
    newMatrix.c[3] = newMatrix.c[0];
    
    *this *= newMatrix;
}

/* Scale matrix by separate x, y, and z amounts */
void Matrix2x3::scale(float x, float y) {
    Matrix2x3 newMatrix; // Create new identity matrix
    newMatrix.init();
    
    newMatrix.c[0] = x;
    newMatrix.c[3] = y;
    
    *this *= newMatrix;
}

void Matrix2x3::scale(const Vector2 &v) {
    scale(v.x, v.y);
}

/* Scale all dimensions equally */
void Matrix2x3::scale(float s) {
    scale(s,s);
}
/* Replace matrix with its inverse */
bool Matrix2x3::invert() {
    float det = c[0] * c[3] - c[1] * c[2];
    if (det == 0) {
        return false;
    }
    float invdet = 1.0f / det;
    float tmp[6];
    tmp[0] = c[3] * invdet;
    tmp[1] = -c[1] * invdet;
    tmp[2] = -c[2] * invdet;
    tmp[3] = c[0] * invdet;
    tmp[4] = (c[2] * c[5] - c[3] * c[4]) * invdet;
    tmp[5] = (c[1] * c[4] - c[0] * c[5]) * invdet;
    memcpy(c, tmp, sizeof(float) * 6);
    
    return true;
}

/* Dot Product, returning Vector2 */
Vector2 Matrix2x3::Dot(const Matrix2x3 &m, const Vector2 &v) {
    return Vector2::Create(
        v.c[0] * m.c[0] + v.c[1] * m.c[2] + m.c[4],
        v.c[0] * m.c[1] + v.c[1] * m.c[3] + m.c[5]
    );
}

// Dot product without including translation; useful for transforming normals and tangents
Vector2 Matrix2x3::DotNoTranslate(const Matrix2x3 &m, const Vector2 &v)
{
    return Vector2::Create(
        v.c[0] * m.c[0] + v.c[1] * m.c[2],
        v.c[0] * m.c[1] + v.c[1] * m.c[3]
    );
}

Matrix2x3 Matrix2x3::Invert(const Matrix2x3 &m)
{
    Matrix2x3 matInvert = m;
    matInvert.invert();
    return matInvert;
}

Matrix2x3 Matrix2x3::Translation(const Vector2 &v)
{
    Matrix2x3 m;
    m.init();
    m.transform.x = v.x;
    m.transform.y = v.y;
    return m;
}

Matrix2x3 Matrix2x3::Rotation(float angle)
{
    Matrix2x3 m;
    m.init();
    m.rotate(angle);
    return m;
}

Matrix2x3 Matrix2x3::Scaling(const Vector2 &v)
{
    Matrix2x3 m;
    m.init();
    m.scale(v);
    return m;
}

Matrix2x3 Matrix2x3::Identity()
{
    Matrix2x3 m;
    m.init();
    return m;
}

} // namespace kraken

