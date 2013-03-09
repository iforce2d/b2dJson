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
#include "b2dJsonImage_OpenGL.h"
#include "bitmap.h"

using namespace std;

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

b2dJsonImage_OpenGL::b2dJsonImage_OpenGL(const b2dJsonImage *other) : b2dJsonImage(other)
{
    m_textureId = 0;
    loadImage();
}

bool checkFileExtension(string s, string ext) 
{
	int extStartPos = s.length() - ext.length();
	if ( extStartPos <= 0 )
		return false;
	return s.substr( extStartPos ) == ext;	
}

bool b2dJsonImage_OpenGL::loadImage()
{
    //clear any existing texture
    if ( m_textureId ) {
        glDeleteTextures(1, &m_textureId);
        m_textureId = 0;
    }

    if ( checkFileExtension(file, ".bmp") || checkFileExtension(file, ".BMP") )
    {
        //use bitmap.h to load bitmap with transparency (https://bugreports.qt-project.org/browse/QTBUG-22031)
        CBitmap bitmap;
        if ( ! bitmap.Load(file.c_str()) ) {
            printf("Could not load image file '%s'\n", file.c_str());
            return false;
        }

        int width = bitmap.GetWidth();
        int height = bitmap.GetHeight();
        unsigned int pixelSize = 0;
        bitmap.GetBits(NULL, pixelSize, 0,0,0,0);//get necessary buffer size
        GLubyte* pixels = new unsigned char[width*height*pixelSize];
        bitmap.GetBits(pixels, pixelSize, 0x000000FF, 0x0000FF00, 0x00FF0000, 0xFF000000);
        glGenTextures(1, &m_textureId);
        glBindTexture(GL_TEXTURE_2D, m_textureId);
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
        if ( filter == FT_LINEAR )
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
        else
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
        delete[] pixels;

        return true;
    }

    return false;
}

void b2dJsonImage_OpenGL::render()
{
    glEnable(GL_TEXTURE_2D);

    glEnable(GL_BLEND);

    glBindTexture(GL_TEXTURE_2D, m_textureId);

    float useRed = colorTint[0] / 255.0f;
    float useGreen = colorTint[1] / 255.0f;
    float useBlue = colorTint[2] / 255.0f;
    float useAlpha = colorTint[3] / 255.0f;
    useAlpha *= opacity; //this will use opacity as defined in RUBE

    glColor4f(useRed,useGreen,useBlue,useAlpha);

    glPushMatrix();

    if ( body ) {
        b2Vec2 bodyPos = body->GetPosition();
        glTranslatef(bodyPos.x, bodyPos.y, 0);
        glRotatef(body->GetAngle()*RADTODEG, 0,0,1);
    }

    //renderUsingCorners();
    renderUsingArrays();

    glPopMatrix();

    glDisable(GL_TEXTURE_2D);
}

void b2dJsonImage_OpenGL::renderUsingCorners()
{
    float lx = 0, ux = 1;
    if ( flip ) {
        lx = 1;
        ux = 0;
    }

    glBegin(GL_TRIANGLE_FAN);
    glTexCoord2f(lx,0);
    glVertex2f(corners[0].x, corners[0].y);
    glTexCoord2f(ux,0);
    glVertex2f(corners[1].x, corners[1].y);
    glTexCoord2f(ux,1);
    glVertex2f(corners[2].x, corners[2].y);
    glTexCoord2f(lx,1);
    glVertex2f(corners[3].x, corners[3].y);
    glEnd();
}

void b2dJsonImage_OpenGL::renderUsingArrays()
{
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);

    glTexCoordPointer(2, GL_FLOAT, 0, uvCoords);
    glVertexPointer(2, GL_FLOAT, 0, points);
    glDrawElements(GL_TRIANGLES, numIndices, GL_UNSIGNED_SHORT, indices);

    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}




