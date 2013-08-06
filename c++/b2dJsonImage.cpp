/*
* Author: Chris Campbell - www.iforce2d.net
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

#include <cstring>
#include <Box2D/Box2D.h>
#include "b2dJsonImage.h"
#include "bitmap.h"

// helper functions

void _setMat33Translation(b2Mat33& mat, b2Vec2 t)
{
    mat.SetZero();
    mat.ex.x = 1;
    mat.ey.y = 1;
    mat.ez.x = t.x;
    mat.ez.y = t.y;
    mat.ez.z = 1;
}

void _setMat33Rotation(b2Mat33& mat, float angle)
{
    mat.SetZero();
    float32 c = cosf(angle), s = sinf(angle);
    mat.ex.x = c; mat.ey.x = -s;
    mat.ex.y = s; mat.ey.y = c;
    mat.ez.z = 1;
}

void _setMat33Scale(b2Mat33& mat, float xfactor, float yfactor)
{
    mat.SetZero();
    mat.ex.x = xfactor;
    mat.ey.y = yfactor;
    mat.ez.z = 1;
}

b2Vec2 _b2Mul(const b2Mat33& A, const b2Vec2& v2)
{
    b2Vec3 v(v2.x, v2.y, 1);
    b2Vec3 r = v.x * A.ex + v.y * A.ey + v.z * A.ez;
    return b2Vec2(r.x, r.y);
}

b2Mat33 _b2Mul(const b2Mat33& B, const b2Mat33& A)
{
    return b2Mat33(b2Mul(A, B.ex), b2Mul(A, B.ey), b2Mul(A, B.ez));
}

///////////////////

b2dJsonImage::b2dJsonImage()
{
    body = NULL;
    center.SetZero();
    angle = 0;
    scale = 1;
    aspectScale = 1;
    flip = false;
    filter = FT_LINEAR;
    opacity = 1;
    renderOrder = 0;
    for (int i = 0; i < 4; i++)
        colorTint[i] = 255;

    numPoints = 0;
    points = NULL;
    uvCoords = NULL;
    numIndices = 0;
    indices = NULL;
}

b2dJsonImage::~b2dJsonImage()
{
    if (points) delete[] points;
    if (uvCoords) delete[] uvCoords;
    if (indices) delete[] indices;
}

b2dJsonImage::b2dJsonImage(const b2dJsonImage *other)
{
    body = other->body;
    name = other->name;
    file = other->file;
    center = other->center;
    angle = other->angle;
    scale = other->scale;
    aspectScale = other->aspectScale;
    flip = other->flip;
    filter = other->filter;
    opacity = other->opacity;
    renderOrder = other->renderOrder;
    for (int i = 0; i < 4; i++)
        colorTint[i] = other->colorTint[i];

    memcpy(corners, other->corners, 4 * sizeof(b2Vec2));

    numPoints = other->numPoints;
    points = new float[2 * numPoints];
    uvCoords = new float[2 * numPoints];
    memcpy(points, other->points, 2 * numPoints * sizeof(float));
    memcpy(uvCoords, other->uvCoords, 2 * numPoints * sizeof(float));
    numIndices = other->numIndices;
    indices = new unsigned short[numIndices];
    memcpy(indices, other->indices, numIndices * sizeof(unsigned short));
}

void b2dJsonImage::updateCorners(float aspect)
{
    float hx = 0.5 * aspect;
    float hy = 0.5;

    corners[0].Set(-hx, -hy);
    corners[1].Set( hx, -hy);
    corners[2].Set( hx,  hy);
    corners[3].Set(-hx,  hy);

    b2Mat33 r, s;
    _setMat33Rotation(r, angle);
    _setMat33Scale(s, scale, scale);
    b2Mat33 m = _b2Mul(r,s);

    for (int i = 0; i < 4; i++) {
        corners[i] = _b2Mul(m, corners[i]);
        corners[i] += center;
    }
}

void b2dJsonImage::updateUVs(float aspect)
{
    //set up vertices

    float hx = 0.5 * aspect;
    float hy = 0.5;

    b2Vec2 verts[4];
    verts[0].Set(-hx, -hy);
    verts[1].Set( hx, -hy);
    verts[2].Set( hx,  hy);
    verts[3].Set(-hx,  hy);

    b2Mat33 r, s;
    _setMat33Rotation(r, angle);
    _setMat33Scale(s, scale, scale);
    b2Mat33 m = _b2Mul(r,s);

    for (int i = 0; i < 4; i++) {
        verts[i] = _b2Mul(m, verts[i]);
        verts[i] += center;
    }

    //set up uvs

    b2Vec2 uvs[4];
    uvs[0].Set(0, 0);
    uvs[1].Set(1, 0);
    uvs[2].Set(1, 1);
    uvs[3].Set(0, 1);

    //set up arrays for rendering

    numPoints = 4;
    numIndices = 6;

    if (points) delete[] points;
    if (uvCoords) delete[] uvCoords;
    if (indices) delete[] indices;
    points = new float[2 * numPoints];
    uvCoords = new float[2 * numPoints];
    indices = new unsigned short[numIndices];

    for (int i = 0; i < numPoints; i++) {
        points[2*i+0] = verts[i].x;
        points[2*i+1] = verts[i].y;
        uvCoords[2*i+0] = uvs[i].x;
        uvCoords[2*i+1] = uvs[i].y;
    }
    indices[0] = 0;
    indices[1] = 1;
    indices[2] = 2;
    indices[3] = 2;
    indices[4] = 3;
    indices[5] = 0;

}

b2AABB b2dJsonImage::getAABB()
{
    b2Mat33 r, t, m;
    if ( body ) {
        _setMat33Rotation(r, body->GetAngle());
        _setMat33Translation(t, body->GetPosition());
        m = _b2Mul(r,t);
    }
    else
        m = b2Mat33( b2Vec3(1,0,0), b2Vec3(0,1,0), b2Vec3(0,0,1) ); //identity matrix

    b2AABB aabb;
    aabb.lowerBound.Set(FLT_MAX, FLT_MAX);
    aabb.upperBound.Set(-FLT_MAX, -FLT_MAX);
    for (int i = 0; i < 4; i++) {
        aabb.lowerBound = b2Min(aabb.lowerBound, _b2Mul(m, corners[i]));
        aabb.upperBound = b2Max(aabb.upperBound, _b2Mul(m, corners[i]));
    }
    return aabb;
}




