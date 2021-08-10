//
//  matrix2.cpp
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

#include "../include/hydra.h"

#include <string.h>

namespace kraken {

void Matrix2::init() {
    // Default constructor - Initialize with an identity matrix
    static const float IDENTITY_MATRIX[] = {
        1.0, 0.0,
        0.0, 1.0
    };
    memcpy(c, IDENTITY_MATRIX, sizeof(float) * 4);
    
}

void Matrix2::init(float *pMat) {
    memcpy(c, pMat, sizeof(float) * 4);
}

void Matrix2::init(const Vector2 &new_axis_x, const Vector2 &new_axis_y)
{
    c[0]  = new_axis_x.x;    c[1]  = new_axis_x.y;
    c[2]  = new_axis_y.x;    c[3]  = new_axis_y.y;
}

void Matrix2::init(const Matrix2 &m) {
    memcpy(c, m.c, sizeof(float) * 4);
}

float *Matrix2::getPointer() {
    return c;
}

float& Matrix2::operator[](unsigned i) {
    return c[i];
}

float Matrix2::operator[](unsigned i) const {
    return c[i];
}

// Overload comparison operator
bool Matrix2::operator==(const Matrix2 &m) const {
    return memcmp(c, m.c, sizeof(float) * 4) == 0;
}

// Overload compound multiply operator
Matrix2& Matrix2::operator*=(const Matrix2 &m) {
    float temp[4];
    
    int x,y;
    
    for (x=0; x < 2; x++)
    {
        for(y=0; y < 2; y++)
        {
            temp[y + (x*2)] = (c[x*2]   * m.c[y]) +
                              (c[x*2+1] * m.c[y+2]);
        }
    }
    
    memcpy(c, temp, sizeof(float) * 4);
    return *this;
}

// Overload multiply operator
Matrix2 Matrix2::operator*(const Matrix2 &m) const {
    Matrix2 ret = *this;
    ret *= m;
    return ret;
}

/* Rotate a matrix by an angle on a X, Y, or Z axis */
void Matrix2::rotate(float angle) {
    
    Matrix2 newMatrix; // Create new identity matrix
    newMatrix.init();
    newMatrix.c[0] = cosf(angle);
    newMatrix.c[1] = -sinf(angle);
    newMatrix.c[2] = -newMatrix.c[1];
    newMatrix.c[3] = newMatrix.c[0];
    
    *this *= newMatrix;
}

/* Scale matrix by separate x, y, and z amounts */
void Matrix2::scale(float x, float y) {
    Matrix2 newMatrix; // Create new identity matrix
    newMatrix.init();
    
    newMatrix.c[0] = x;
    newMatrix.c[3] = y;
    
    *this *= newMatrix;
}

void Matrix2::scale(const Vector2 &v) {
    scale(v.x, v.y);
}

/* Scale all dimensions equally */
void Matrix2::scale(float s) {
    scale(s,s);
}

/* Replace matrix with its inverse */
bool Matrix2::invert() {
    float det = c[0] * c[3] - c[1] * c[2];
    if (det == 0) {
        return false;
    }
    float invdet = 1.0f / det;
    float tmp = c[0];
    c[0] = c[3] * invdet;
    c[1] = -c[1] * invdet;
    c[2] = -c[2] * invdet;
    c[3] = tmp * invdet;
    
    return true;
}

void Matrix2::transpose() {
    float tmp = c[1];
    c[1] = c[2];
    c[2] = tmp;
}

/* Dot Product, returning Vector2 */
Vector2 Matrix2::Dot(const Matrix2 &m, const Vector2 &v) {
    return Vector2::Create(
        v.c[0] * m.c[0] + v.c[1] * m.c[2],
        v.c[0] * m.c[1] + v.c[1] * m.c[3]
    );
}

Matrix2 Matrix2::Invert(const Matrix2 &m)
{
    Matrix2 matInvert = m;
    matInvert.invert();
    return matInvert;
}

Matrix2 Matrix2::Transpose(const Matrix2 &m)
{
    Matrix2 matTranspose = m;
    matTranspose.transpose();
    return matTranspose;
}

Matrix2 Matrix2::Rotation(float angle)
{
    Matrix2 m;
    m.init();
    m.rotate(angle);
    return m;
}

Matrix2 Matrix2::Scaling(const Vector2 &v)
{
    Matrix2 m;
    m.init();
    m.scale(v);
    return m;
}

Matrix2 Matrix2::Identity()
{
    Matrix2 m;
    m.init();
    return m;
}

} // namespace kraken

