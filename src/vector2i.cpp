//
//  Vector2i.cpp
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

#include "../include/kraken-math.h"

namespace kraken {

void Vector2i::init() {
    x = 0;
    y = 0;
}

Vector2i Vector2i::Create()
{
    Vector2i r;
    r.init();
    return r;
}

void Vector2i::init(int X, int Y) {
    x = X;
    y = Y;
}

Vector2i Vector2i::Create(int X, int Y)
{
    Vector2i r;
    r.init(X,Y);
    return r;
}

void Vector2i::init(int v) {
    x = v;
    y = v;
}

Vector2i Vector2i::Create(int v)
{
    Vector2i r;
    r.init(v);
    return r;
}

void Vector2i::init(int *v) {
    x = v[0];
    y = v[1];
}

Vector2i Vector2i::Create(int *v)
{
    Vector2i r;
    r.init(v);
    return r;
}

void Vector2i::init(const Vector2i &v) {
    x = v.x;
    y = v.y;
}

Vector2i Vector2i::Create(const Vector2i &v)
{
    Vector2i r;
    r.init(v);
    return r;
}

// Vector2 swizzle getters
Vector2i Vector2i::yx() const
{
    return Vector2i::Create(y,x);
}

// Vector2 swizzle setters
void Vector2i::yx(const Vector2i &v)
{
    y = v.x;
    x = v.y;
}

Vector2i Vector2i::Min() {
    return Vector2i::Create(-std::numeric_limits<int>::max());
}

Vector2i Vector2i::Max() {
    return Vector2i::Create(std::numeric_limits<int>::max());
}

Vector2i Vector2i::Zero() {
    return Vector2i::Create(0);
}

Vector2i Vector2i::One() {
    return Vector2i::Create(1);
}

Vector2i Vector2i::operator +(const Vector2i& b) const {
    return Vector2i::Create(x + b.x, y + b.y);
}

Vector2i Vector2i::operator -(const Vector2i& b) const {
    return Vector2i::Create(x - b.x, y - b.y);
}

Vector2i Vector2i::operator +() const {
    return *this;
}

Vector2i Vector2i::operator -() const {
    return Vector2i::Create(-x, -y);
}

Vector2i Vector2i::operator *(const int v) const {
    return Vector2i::Create(x * v, y * v);
}

Vector2i Vector2i::operator /(const int v) const {
    return Vector2i::Create(x / v, y / v);
}

Vector2i& Vector2i::operator +=(const Vector2i& b) {
    x += b.x;
    y += b.y;
    return *this;
}

Vector2i& Vector2i::operator -=(const Vector2i& b) {
    x -= b.x;
    y -= b.y;
    return *this;
}

Vector2i& Vector2i::operator *=(const int v) {
    x *= v;
    y *= v;
    return *this;
}

Vector2i& Vector2i::operator /=(const int v) {
    x /= v;
    y /= v;
    return *this;
}

bool Vector2i::operator ==(const Vector2i& b) const {
    return x == b.x && y == b.y;
}

bool Vector2i::operator !=(const Vector2i& b) const {
    return x != b.x || y != b.y;
}

bool Vector2i::operator >(const Vector2i& b) const
{
    // Comparison operators are implemented to allow insertion into sorted containers such as std::set
    if(x > b.x) {
        return true;
    } else if(x < b.x) {
        return false;
    } else if(y > b.y) {
        return true;
    } else {
        return false;
    }
}

bool Vector2i::operator <(const Vector2i& b) const
{
    // Comparison operators are implemented to allow insertion into sorted containers such as std::set
    if(x < b.x) {
        return true;
    } else if(x > b.x) {
        return false;
    } else if(y < b.y) {
        return true;
    } else {
        return false;
    }
}

int& Vector2i::operator[] (unsigned i) {
    switch(i) {
        case 0:
            return x;
        case 1:
        default:
            return y;
    }
}

int Vector2i::operator[](unsigned i) const {
    switch(i) {
        case 0:
            return x;
        case 1:
        default:
            return y;
    }
}

void Vector2i::normalize() {
    int m = magnitude();
    x /= m;
    y /= m;
}

int Vector2i::sqrMagnitude() const {
    return x * x + y * y;
}

int Vector2i::magnitude() const {
    return static_cast<int>(sqrt(x * x + y * y));
}

Vector2i Vector2i::Normalize(const Vector2i &v) {
    int m = v.magnitude();
    return Vector2i::Create(v.x / m, v.y / m);
}

int Vector2i::Cross(const Vector2i &v1, const Vector2i &v2) {
    return v1.x * v2.y - v1.y * v2.x;
}

int Vector2i::Dot(const Vector2i &v1, const Vector2i &v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

} // namepsace kraken
