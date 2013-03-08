/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_MATH_H
#define B2_MATH_H

//#include <cmath>
//#include <cstddef>

typedef float float32;

//for now, can keep this at 8 because RUBE never
//outputs polygons with more than 8 vertices
#define b2_maxPolygonVertices 8

/// A 2D column vector with 3 elements.
struct b2Vec3
{
    /// Default constructor does nothing (for performance).
    b2Vec3() {}

    /// Construct using coordinates.
    b2Vec3(float32 x, float32 y, float32 z) : x(x), y(y), z(z) {}

    /// Set this vector to all zeros.
    void SetZero() { x = 0.0f; y = 0.0f; z = 0.0f; }

    /// Set this vector to some specified coordinates.
    void Set(float32 x_, float32 y_, float32 z_) { x = x_; y = y_; z = z_; }

    /// Negate this vector.
    b2Vec3 operator -() const { b2Vec3 v; v.Set(-x, -y, -z); return v; }

    /// Add a vector to this vector.
    void operator += (const b2Vec3& v)
    {
        x += v.x; y += v.y; z += v.z;
    }

    /// Subtract a vector from this vector.
    void operator -= (const b2Vec3& v)
    {
        x -= v.x; y -= v.y; z -= v.z;
    }

    b2Vec3 operator *(const float32 s)
    {
        return b2Vec3( s*x, s*y, s*z );
    }

    /// Multiply this vector by a scalar.
    void operator *= (float32 s)
    {
        x *= s; y *= s; z *= s;
    }

    float32 x, y, z;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
    /// The default constructor does nothing (for performance).
    b2Mat33() {}

    /// Construct this matrix using columns.
    b2Mat33(const b2Vec3& c1, const b2Vec3& c2, const b2Vec3& c3)
    {
        ex = c1;
        ey = c2;
        ez = c3;
    }

    /// Set this matrix to all zeros.
    void SetZero()
    {
        ex.SetZero();
        ey.SetZero();
        ez.SetZero();
    }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    //b2Vec3 Solve33(const b2Vec3& b) const;

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases. Solve only the upper
    /// 2-by-2 matrix equation.
    //b2Vec2 Solve22(const b2Vec2& b) const;

    /// Get the inverse of this matrix as a 2-by-2.
    /// Returns the zero matrix if singular.
    //void GetInverse22(b2Mat33* M) const;

    /// Get the symmetric inverse of this matrix as a 3-by-3.
    /// Returns the zero matrix if singular.
    //void GetSymInverse33(b2Mat33* M) const;

    b2Vec3 ex, ey, ez;
};

/// Add two vectors component-wise.
inline b2Vec3 operator + (const b2Vec3& a, const b2Vec3& b)
{
    return b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline b2Vec3 operator * (float32 s, const b2Vec3& a)
{
    return b2Vec3(s * a.x, s * a.y, s * a.z);
}

/// Multiply a matrix times a vector.
inline b2Vec3 b2Mul(const b2Mat33& A, const b2Vec3& v)
{
    return v.x * A.ex + v.y * A.ey + v.z * A.ez;
}

#endif

