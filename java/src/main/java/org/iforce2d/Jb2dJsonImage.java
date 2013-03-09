package org.iforce2d;

/*
 Author: Chris Campbell - www.iforce2d.net

 This software is provided 'as-is', without any express or implied
 warranty.  In no event will the authors be held liable for any damages
 arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it
 freely, subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
*/

import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;

import org.jbox2d.collision.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.*;

import org.json.*;

/**
 * 
 *
 */
public class Jb2dJsonImage {

    String name;
    String file;
    Body body;
    Vec2 center;
    float angle;
    float scale;
    boolean flip;
    float opacity;
    int filter; // 0 = nearest, 1 = linear
    float renderOrder;
    int colorTint[];

    Vec2 corners[];

    int numPoints;
    float points[];
    float uvCoords[];
    int numIndices;
    short indices[];

    public Jb2dJsonImage() {
    	colorTint = new int[4];
    }
    
}

