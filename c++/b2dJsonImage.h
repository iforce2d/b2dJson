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

#ifndef B2DJSONIMAGE_H
#define B2DJSONIMAGE_H


#include <string>
#include <Box2D/Box2D.h>

enum _b2dJsonImagefilterType {
    FT_NEAREST,
    FT_LINEAR,

    FT_MAX
};

class b2dJsonImage {
public:
    std::string name;
    std::string file;
    std::string path;
    b2Body* body;
    b2Vec2 center;
    float angle;
    float scale;
    float aspectScale;
    bool flip;
    float opacity;
    int filter; // 0 = nearest, 1 = linear
    float renderOrder;
    int colorTint[4];

    b2Vec2 corners[4];

    int numPoints;
    float* points;
    float* uvCoords;
    int numIndices;
    unsigned short* indices;

    b2dJsonImage();
    ~b2dJsonImage();
    b2dJsonImage(const b2dJsonImage* other);

    void updateCorners(float aspect);
    void updateUVs(float aspect);
    b2AABB getAABB();

    virtual bool loadImage() { return false; }
    virtual void render() {}
};

#endif // B2DJSONIMAGE_H
