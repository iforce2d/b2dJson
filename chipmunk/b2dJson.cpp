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

#include <istream>
#include <fstream>
#include <algorithm>
#include "b2dJson.h"
#include "json/json.h"
#include "b2dJsonImage.h"

#include "b2Math_excerpts.h"

using namespace std;

inline bool b2dJsonImage_renderOrder_ascending(const b2dJsonImage* a, const b2dJsonImage* b)
{
    return a->renderOrder < b->renderOrder;
}

b2dJson::b2dJson(bool useHumanReadableFloats)
{
    m_useHumanReadableFloats = useHumanReadableFloats;
    m_simulationPositionIterations = 3;
    m_simulationVelocityIterations = 8;
    m_simulationFPS = 60;
}

b2dJson::~b2dJson()
{
    for (int i = 0; i < (int)m_images.size(); i++)
        delete m_images[i];
    for (std::map<void*,b2dJsonCustomProperties*>::iterator it = m_customPropertiesMap.begin(); it != m_customPropertiesMap.end(); ++it)
        delete it->second;
}

void b2dJson::setSimulationPositionIterations(int n)
{
    m_simulationPositionIterations = n;
}

void b2dJson::setSimulationVelocityIterations(int n)
{
    m_simulationVelocityIterations = n;
}

void b2dJson::setSimulationFPS(float n)
{
    m_simulationFPS = n;
}

int b2dJson::getSimulationPositionIterations()
{
    return m_simulationPositionIterations;
}

int b2dJson::getSimulationVelocityIterations()
{
    return m_simulationVelocityIterations;
}

float b2dJson::getSimulationFPS()
{
    return m_simulationFPS;
}

/*
Json::Value b2dJson::writeToValue(cpSpace *world)
{
    if (!world)
        return Json::Value();

    return b2j(world);
}

std::string b2dJson::writeToString(cpSpace* world)
{
    if (!world)
        return std::string();

    Json::StyledWriter writer;
    return writer.write( b2j(world) );
}

bool b2dJson::writeToFile(cpSpace* world, const char* filename)
{
    if ( !world || !filename )
        return false;

    std::ofstream ofs;
    ofs.open(filename, std::ios::out);
    if (!ofs) {
        std::cout << "Could not open file " << filename << " for writing\n";
        return false;
    }

    Json::StyledStreamWriter writer("   ");
    writer.write( ofs, b2j(world) );

    ofs.close();

    return true;
}

Json::Value b2dJson::b2j(cpSpace* world)
{
    m_bodyToIndexMap.clear();
    m_jointToIndexMap.clear();

    Json::Value worldValue;

    vecToJson("gravity", world->GetGravity(), worldValue);
    worldValue["positionIterations"] = m_simulationPositionIterations;
    worldValue["velocityIterations"] = m_simulationVelocityIterations;
    worldValue["stepsPerSecond"] = m_simulationFPS;
    worldValue["allowSleep"] = world->GetAllowSleeping();
    worldValue["autoClearForces"] = world->GetAutoClearForces();
    worldValue["warmStarting"] = world->GetWarmStarting();
    worldValue["continuousPhysics"] = world->GetContinuousPhysics();
    worldValue["subStepping"] = world->GetSubStepping();
    //worldValue["hasDestructionListener"] = world->HasDestructionListener();
    //worldValue["hasContactFilter"] = world->HasContactFilter();
    //worldValue["hasContactListener"] = world->HasContactListener();

    int i =  0;
    for (cpBody* body = world->GetBodyList(); body; body = body->GetNext()) {
        m_bodyToIndexMap[body] = i;
        worldValue["body"][i] =  b2j(body);
        i++;
    }

    //need two passes for joints because gear joints reference other joints
    i = 0;
    for (cpConstraint* joint = world->GetJointList(); joint; joint = joint->GetNext()) {
        if ( joint->GetType() == e_gearJoint )
            continue;
        worldValue["joint"][i] =  b2j(joint);
        m_jointToIndexMap[joint] = i;
        i++;
    }
    for (cpConstraint* joint = world->GetJointList(); joint; joint = joint->GetNext()) {
        if ( joint->GetType() != e_gearJoint )
            continue;
        worldValue["joint"][i] =  b2j(joint);
        m_jointToIndexMap[joint] = i;
        i++;
    }

    i = 0;
    {
        std::map<b2dJsonImage*,string>::iterator it = m_imageToNameMap.begin();
        std::map<b2dJsonImage*,string>::iterator end = m_imageToNameMap.end();
        while (it != end) {
            b2dJsonImage* image = it->first;
            worldValue["image"][i] =  b2j(image);
            i++;

            ++it;
        }
    }

    Json::Value customPropertyValue = writeCustomPropertiesToJson(NULL);
    if ( ! customPropertyValue.empty() )
        worldValue["customProperties"] = customPropertyValue;

    m_bodyToIndexMap.clear();
    m_jointToIndexMap.clear();

    return worldValue;
}

Json::Value b2dJson::b2j(cpBody* body)
{
    Json::Value bodyValue;

    string bodyName = getBodyName(body);
    if ( bodyName != "" )
        bodyValue["name"] = bodyName;

    bodyValue["type"] = body->GetType();
    switch( body->GetType() )
    {
    case b2_staticBody:
        bodyValue["type"].setComment("//static", Json::commentAfterOnSameLine);
        break;
    case b2_dynamicBody:
        bodyValue["type"].setComment("//dynamic", Json::commentAfterOnSameLine);
        break;
    case b2_kinematicBody:
        bodyValue["type"].setComment("//kinematic", Json::commentAfterOnSameLine);
        break;
    }

    vecToJson("position", body->GetPosition(), bodyValue);
    floatToJson("angle", body->GetAngle(), bodyValue );

    vecToJson("linearVelocity", body->GetLinearVelocity(), bodyValue);
    floatToJson("angularVelocity", body->GetAngularVelocity(), bodyValue);

    if ( body->GetLinearDamping() != 0 )
        floatToJson("linearDamping", body->GetLinearDamping(), bodyValue);
    if ( body->GetAngularDamping() != 0 )
        floatToJson("angularDamping", body->GetAngularDamping(), bodyValue);
    if ( body->GetGravityScale() != 1 )
        floatToJson("gravityScale", body->GetGravityScale(), bodyValue);

    if ( body->IsBullet() )
        bodyValue["bullet"] = true;
    if ( ! body->IsSleepingAllowed() )
        bodyValue["allowSleep"] = false;
    if ( body->IsAwake() )
        bodyValue["awake"] = true;
    if ( ! body->IsActive() )
        bodyValue["active"] = false;
    if ( body->IsFixedRotation() )
        bodyValue["fixedRotation"] = true;

    b2MassData massData;
    body->GetMassData(&massData);
    if ( massData.mass != 0 )
        floatToJson("massData-mass", massData.mass, bodyValue);
    if ( massData.center.x != 0 || massData.center.y != 0 )
        vecToJson("massData-center", massData.center, bodyValue);
    if ( massData.I != 0 )
        floatToJson("massData-I", massData.I, bodyValue);

    int i = 0;
    for (cpShape* fixture = body->GetFixtureList(); fixture; fixture = fixture->GetNext())
        bodyValue["fixture"][i++] = b2j(fixture);

    Json::Value customPropertyValue = writeCustomPropertiesToJson(body);
    if ( ! customPropertyValue.empty() )
        bodyValue["customProperties"] = customPropertyValue;

    return bodyValue;
}

Json::Value b2dJson::b2j(cpShape *fixture)
{
    Json::Value fixtureValue;

    string fixtureName = getFixtureName(fixture);
    if ( fixtureName != "" )
        fixtureValue["name"] = fixtureName;

    if ( fixture->GetRestitution() != 0 )
        floatToJson("restitution", fixture->GetRestitution(), fixtureValue);
    if ( fixture->GetFriction() != 0 )
        floatToJson("friction", fixture->GetFriction(), fixtureValue);
    if ( fixture->GetDensity() != 0 )
        floatToJson("density", fixture->GetDensity(), fixtureValue);
    if ( fixture->IsSensor() )
        fixtureValue["sensor"] = true;

    b2Filter filter = fixture->GetFilterData();
    if ( filter.categoryBits != 0x0001 )
        fixtureValue["filter-categoryBits"] = filter.categoryBits;
    if ( filter.maskBits != 0xffff )
        fixtureValue["filter-maskBits"] = filter.maskBits;
    if ( filter.groupIndex != 0 )
        fixtureValue["filter-groupIndex"] = filter.groupIndex;

    b2Shape* shape = fixture->GetShape();
    switch (shape->GetType())
    {
    case b2Shape::e_circle:
        {
            b2CircleShape* circle = (b2CircleShape*)shape;
            floatToJson("radius", circle->m_radius, fixtureValue["circle"]);
            vecToJson("center", circle->m_p, fixtureValue["circle"]);
        }
        break;
    case b2Shape::e_edge:
        {
            b2EdgeShape* edge = (b2EdgeShape*)shape;
            vecToJson("vertex1", edge->m_vertex1, fixtureValue["edge"]);
            vecToJson("vertex2", edge->m_vertex2, fixtureValue["edge"]);
            if ( edge->m_hasVertex0 )
                fixtureValue["edge"]["hasVertex0"] = true;
            if ( edge->m_hasVertex3 )
                fixtureValue["edge"]["hasVertex3"] = true;
            if ( edge->m_hasVertex0 )
                vecToJson("vertex0", edge->m_vertex0, fixtureValue["edge"]);
            if ( edge->m_hasVertex3 )
                vecToJson("vertex3", edge->m_vertex3, fixtureValue["edge"]);
        }
        break;
    case b2Shape::e_chain:
        {
            b2ChainShape* chain = (b2ChainShape*)shape;
            int32 count = chain->m_count;
            const cpVect* vertices = chain->m_vertices;
            for (int32 i = 0; i < count; ++i)
                vecToJson("vertices", vertices[i], fixtureValue["chain"], i);
            if ( chain->m_hasPrevVertex )
                fixtureValue["chain"]["hasPrevVertex"] = true;
            if ( chain->m_hasNextVertex )
                fixtureValue["chain"]["hasNextVertex"] = true;
            if ( chain->m_hasPrevVertex )
                vecToJson("prevVertex", chain->m_prevVertex, fixtureValue["chain"]);
            if ( chain->m_hasNextVertex )
                vecToJson("nextVertex", chain->m_nextVertex, fixtureValue["chain"]);
        }
        break;
    case b2Shape::e_polygon:
        {
            b2PolygonShape* poly = (b2PolygonShape*)shape;
            int32 vertexCount = poly->GetVertexCount();
            b2Assert(vertexCount <= b2_maxPolygonVertices);
            for (int32 i = 0; i < vertexCount; ++i)
                vecToJson("vertices", poly->m_vertices[i], fixtureValue["polygon"], i);
        }
        break;
    default:
        std::cout << "Unknown shape type : " << shape->GetType() << std::endl;
    }

    Json::Value customPropertyValue = writeCustomPropertiesToJson(fixture);
    if ( ! customPropertyValue.empty() )
        fixtureValue["customProperties"] = customPropertyValue;

    return fixtureValue;
}

Json::Value b2dJson::b2j(cpConstraint* joint)
{
    Json::Value jointValue;

    int bodyIndexA = lookupBodyIndex( joint->GetBodyA() );
    int bodyIndexB = lookupBodyIndex( joint->GetBodyB() );
    jointValue["bodyA"] = bodyIndexA;
    jointValue["bodyB"] = bodyIndexB;
    if ( joint->GetCollideConnected() )
        jointValue["collideConnected"] = true;

    string jointName = getJointName(joint);
    if ( jointName != "" )
        jointValue["name"] = jointName;

    cpBody* bodyA = joint->GetBodyA();
    cpBody* bodyB = joint->GetBodyB();

    switch ( joint->GetType() )
    {
    case e_revoluteJoint:
        {
            jointValue["type"] = "revolute";

            b2RevoluteJoint* revoluteJoint = (b2RevoluteJoint*)joint;
            vecToJson("anchorA", bodyA->GetLocalPoint(revoluteJoint->GetAnchorA()), jointValue);
            vecToJson("anchorB", bodyB->GetLocalPoint(revoluteJoint->GetAnchorB()), jointValue);
            floatToJson("refAngle", bodyB->GetAngle() - bodyA->GetAngle() - revoluteJoint->GetJointAngle(), jointValue);
            floatToJson("jointSpeed", revoluteJoint->GetJointSpeed(), jointValue);
            jointValue["enableLimit"] = revoluteJoint->IsLimitEnabled();
            floatToJson("lowerLimit", revoluteJoint->GetLowerLimit(), jointValue);
            floatToJson("upperLimit", revoluteJoint->GetUpperLimit(), jointValue);
            jointValue["enableMotor"] = revoluteJoint->IsMotorEnabled();
            floatToJson("motorSpeed", revoluteJoint->GetMotorSpeed(), jointValue);
            floatToJson("maxMotorTorque", revoluteJoint->GetMaxMotorTorque(), jointValue);
        }
        break;
    case e_prismaticJoint:
        {
            jointValue["type"] = "prismatic";

            b2PrismaticJoint* prismaticJoint = (b2PrismaticJoint*)joint;
            vecToJson("anchorA", bodyA->GetLocalPoint(prismaticJoint->GetAnchorA()), jointValue);
            vecToJson("anchorB", bodyB->GetLocalPoint(prismaticJoint->GetAnchorB()), jointValue);
            vecToJson("localAxisA", prismaticJoint->GetLocalAxisA(), jointValue);
            floatToJson("refAngle", prismaticJoint->GetReferenceAngle(), jointValue);
            jointValue["enableLimit"] = prismaticJoint->IsLimitEnabled();
            floatToJson("lowerLimit", prismaticJoint->GetLowerLimit(), jointValue);
            floatToJson("upperLimit", prismaticJoint->GetUpperLimit(), jointValue);
            jointValue["enableMotor"] = prismaticJoint->IsMotorEnabled();
            floatToJson("maxMotorForce", prismaticJoint->GetMaxMotorForce(), jointValue);
            floatToJson("motorSpeed", prismaticJoint->GetMotorSpeed(), jointValue);
        }
        break;
    case e_distanceJoint:
        {
            jointValue["type"] = "distance";

            b2DistanceJoint* distanceJoint = (b2DistanceJoint*)joint;
            vecToJson("anchorA", bodyA->GetLocalPoint(distanceJoint->GetAnchorA()), jointValue);
            vecToJson("anchorB", bodyB->GetLocalPoint(distanceJoint->GetAnchorB()), jointValue);
            floatToJson("length", distanceJoint->GetLength(), jointValue);
            floatToJson("frequency", distanceJoint->GetFrequency(), jointValue);
            floatToJson("dampingRatio", distanceJoint->GetDampingRatio(), jointValue);
        }
        break;
    case e_pulleyJoint:
        {
            jointValue["type"] = "pulley";

            b2PulleyJoint* pulleyJoint = (b2PulleyJoint*)joint;
            vecToJson("groundAnchorA", pulleyJoint->GetGroundAnchorA(), jointValue);
            vecToJson("groundAnchorB", pulleyJoint->GetGroundAnchorB(), jointValue);
            vecToJson("anchorA", bodyA->GetLocalPoint(pulleyJoint->GetAnchorA()), jointValue);
            vecToJson("anchorB", bodyB->GetLocalPoint(pulleyJoint->GetAnchorB()), jointValue);
            floatToJson("lengthA", (pulleyJoint->GetGroundAnchorA() - pulleyJoint->GetAnchorA()).Length(), jointValue);
            floatToJson("lengthB", (pulleyJoint->GetGroundAnchorB() - pulleyJoint->GetAnchorB()).Length(), jointValue);
            floatToJson("ratio", pulleyJoint->GetRatio(), jointValue);
        }
        break;
    case e_mouseJoint:
        {
            jointValue["type"] = "mouse";

            b2MouseJoint* mouseJoint = (b2MouseJoint*)joint;
            vecToJson("target", mouseJoint->GetTarget(), jointValue);
            vecToJson("anchorB", mouseJoint->GetAnchorB(), jointValue);
            floatToJson("maxForce", mouseJoint->GetMaxForce(), jointValue);
            floatToJson("frequency", mouseJoint->GetFrequency(), jointValue);
            floatToJson("dampingRatio", mouseJoint->GetDampingRatio(), jointValue);
        }
        break;
    case e_gearJoint:
        {
            jointValue["type"] = "gear";

            b2GearJoint* gearJoint = (b2GearJoint*)joint;
            int jointIndex1 = lookupJointIndex( gearJoint->GetJoint1() );
            int jointIndex2 = lookupJointIndex( gearJoint->GetJoint2() );
            jointValue["joint1"] = jointIndex1;
            jointValue["joint2"] = jointIndex2;
            jointValue["ratio"] = gearJoint->GetRatio();
        }
        break;
    case e_wheelJoint:
        {
            jointValue["type"] = "wheel";

            b2WheelJoint* wheelJoint = (b2WheelJoint*)joint;
            vecToJson("anchorA", bodyA->GetLocalPoint(wheelJoint->GetAnchorA()), jointValue);
            vecToJson("anchorB", bodyB->GetLocalPoint(wheelJoint->GetAnchorB()), jointValue);
            vecToJson("localAxisA", wheelJoint->GetLocalAxisA(), jointValue);
            jointValue["enableMotor"] = wheelJoint->IsMotorEnabled();
            floatToJson("motorSpeed", wheelJoint->GetMotorSpeed(), jointValue);
            floatToJson("maxMotorTorque", wheelJoint->GetMaxMotorTorque(), jointValue);
            floatToJson("springFrequency", wheelJoint->GetSpringFrequencyHz(), jointValue);
            floatToJson("springDampingRatio", wheelJoint->GetSpringDampingRatio(), jointValue);
        }
        break;
    case e_weldJoint:
        {
            jointValue["type"] = "weld";

            b2WeldJoint* weldJoint = (b2WeldJoint*)joint;
            vecToJson("anchorA", bodyA->GetLocalPoint(weldJoint->GetAnchorA()), jointValue);
            vecToJson("anchorB", bodyB->GetLocalPoint(weldJoint->GetAnchorB()), jointValue);
            floatToJson("refAngle", weldJoint->GetReferenceAngle(), jointValue);
        }
        break;
    case e_frictionJoint:
        {
            jointValue["type"] = "friction";

            b2FrictionJoint* frictionJoint = (b2FrictionJoint*)joint;
            vecToJson("anchorA", bodyA->GetLocalPoint(frictionJoint->GetAnchorA()), jointValue);
            vecToJson("anchorB", bodyB->GetLocalPoint(frictionJoint->GetAnchorB()), jointValue);
            floatToJson("maxForce", frictionJoint->GetMaxForce(), jointValue);
            floatToJson("maxTorque", frictionJoint->GetMaxTorque(), jointValue);
        }
        break;
    case e_ropeJoint:
        {
            jointValue["type"] = "rope";

            b2RopeJoint* ropeJoint = (b2RopeJoint*)joint;
            vecToJson("anchorA", bodyA->GetLocalPoint(ropeJoint->GetAnchorA()), jointValue);
            vecToJson("anchorB", bodyB->GetLocalPoint(ropeJoint->GetAnchorB()), jointValue);
            floatToJson("maxLength", ropeJoint->GetMaxLength(), jointValue);
        }
        break;
    case e_unknownJoint:
    default:
        std::cout << "Unknown joint type not stored in snapshot : " << joint->GetType() << std::endl;
    }

    Json::Value customPropertyValue = writeCustomPropertiesToJson(joint);
    if ( ! customPropertyValue.empty() )
        jointValue["customProperties"] = customPropertyValue;

    return jointValue;
}

Json::Value b2dJson::b2j(b2dJsonImage *image)
{
    Json::Value imageValue;

    if ( image->body )
        imageValue["body"] = lookupBodyIndex( image->body );
    else
        imageValue["body"] = -1;

    if ( image->name != "" )
        imageValue["name"] = image->name;
    if ( image->file != "" )
        imageValue["file"] = image->file;

    vecToJson("center", image->center, imageValue);
    floatToJson("angle", image->angle, imageValue );
    floatToJson("scale", image->scale, imageValue );
    if ( image->flip )
        imageValue["flip"] = true;
    floatToJson("opacity", image->opacity, imageValue );
    imageValue["filter"] = image->filter;
    floatToJson("renderOrder", image->renderOrder, imageValue );

    //image->updateCorners();
    for (int i = 0; i < 4; i++)
        vecToJson("corners", image->m_corners[i], imageValue, i);

    //image->updateUVs();
    for (int i = 0; i < 2*image->numPoints; i++) {
        vecToJson("glVertexPointer", image->points[i], imageValue, i);
        vecToJson("glTexCoordPointer", image->uvCoords[i], imageValue, i);
    }
    for (int i = 0; i < image->numIndices; i++)
        vecToJson("glDrawElements", (unsigned int)image->indices[i], imageValue, i);

    Json::Value customPropertyValue = writeCustomPropertiesToJson(image);
    if ( ! customPropertyValue.empty() )
        imageValue["customProperties"] = customPropertyValue;

    return imageValue;
}
*/

void b2dJson::setBodyName(cpBody* body, const char* name)
{
    m_bodyToNameMap[body] = name;
}

void b2dJson::setFixtureName(cpShape* fixture, const char* name)
{
    m_fixtureToNameMap[fixture] = name;
}

void b2dJson::setJointName(cpConstraint* joint, const char* name)
{
    m_jointToNameMap[joint] = name;
}

void b2dJson::setImageName(b2dJsonImage* image, const char* name)
{
    m_imageToNameMap[image] = name;
}

void b2dJson::addImage(b2dJsonImage *image)
{
    setImageName(image, image->name.c_str());
}









/////////

b2dJsonCustomProperties *b2dJson::getCustomPropertiesForItem(void *item, bool createIfNotExisting)
{
    std::map<void*,b2dJsonCustomProperties*>::iterator it = m_customPropertiesMap.find(item);
    if ( it != m_customPropertiesMap.end() )
        return it->second;

    if ( !createIfNotExisting )
        return NULL;

    b2dJsonCustomProperties* props = new b2dJsonCustomProperties();
    m_customPropertiesMap[item] = props;

    return props;
}

void b2dJson::setCustomInt(void* item, string propertyName, int val)        { getCustomPropertiesForItem(item, true)->m_customPropertyMap_int[propertyName] = val; }
void b2dJson::setCustomFloat(void* item, string propertyName, float val)    { getCustomPropertiesForItem(item, true)->m_customPropertyMap_float[propertyName] = val; }
void b2dJson::setCustomString(void* item, string propertyName, string val)  { getCustomPropertiesForItem(item, true)->m_customPropertyMap_string[propertyName] = val; }
void b2dJson::setCustomVector(void* item, string propertyName, cpVect val)  { getCustomPropertiesForItem(item, true)->m_customPropertyMap_cpVect[propertyName] = val; }
void b2dJson::setCustomBool(void* item, string propertyName, bool val)      { getCustomPropertiesForItem(item, true)->m_customPropertyMap_bool[propertyName] = val; }

bool b2dJson::hasCustomInt(void *item, string propertyName)     { return getCustomPropertiesForItem(item, false) != NULL && getCustomPropertiesForItem(item, false)->m_customPropertyMap_int.count(propertyName) > 0; }
bool b2dJson::hasCustomFloat(void *item, string propertyName)   { return getCustomPropertiesForItem(item, false) != NULL && getCustomPropertiesForItem(item, false)->m_customPropertyMap_float.count(propertyName) > 0; }
bool b2dJson::hasCustomString(void *item, string propertyName)  { return getCustomPropertiesForItem(item, false) != NULL && getCustomPropertiesForItem(item, false)->m_customPropertyMap_string.count(propertyName) > 0; }
bool b2dJson::hasCustomVector(void *item, string propertyName)  { return getCustomPropertiesForItem(item, false) != NULL && getCustomPropertiesForItem(item, false)->m_customPropertyMap_cpVect.count(propertyName) > 0; }
bool b2dJson::hasCustomBool(void *item, string propertyName)    { return getCustomPropertiesForItem(item, false) != NULL && getCustomPropertiesForItem(item, false)->m_customPropertyMap_bool.count(propertyName) > 0; }

int b2dJson::getCustomInt(void *item, string propertyName, int defaultVal)
{
    b2dJsonCustomProperties* props = getCustomPropertiesForItem(item, false);
    if ( !props )
        return defaultVal;
    std::map<string,int>::iterator it = props->m_customPropertyMap_int.find(propertyName);
    if ( it != props->m_customPropertyMap_int.end() )
        return it->second;
    return defaultVal;
}

float b2dJson::getCustomFloat(void *item, string propertyName, float defaultVal)
{
    b2dJsonCustomProperties* props = getCustomPropertiesForItem(item, false);
    if ( !props )
        return defaultVal;
    std::map<string,float>::iterator it = props->m_customPropertyMap_float.find(propertyName);
    if ( it != props->m_customPropertyMap_float.end() )
        return it->second;
    return defaultVal;
}

string b2dJson::getCustomString(void *item, string propertyName, string defaultVal)
{
    b2dJsonCustomProperties* props = getCustomPropertiesForItem(item, false);
    if ( !props )
        return defaultVal;
    std::map<string,string>::iterator it = props->m_customPropertyMap_string.find(propertyName);
    if ( it != props->m_customPropertyMap_string.end() )
        return it->second;
    return defaultVal;
}

cpVect b2dJson::getCustomVector(void *item, string propertyName, cpVect defaultVal)
{
    b2dJsonCustomProperties* props = getCustomPropertiesForItem(item, false);
    if ( !props )
        return defaultVal;
    std::map<string,cpVect>::iterator it = props->m_customPropertyMap_cpVect.find(propertyName);
    if ( it != props->m_customPropertyMap_cpVect.end() )
        return it->second;
    return defaultVal;
}

bool b2dJson::getCustomBool(void *item, string propertyName, bool defaultVal)
{
    b2dJsonCustomProperties* props = getCustomPropertiesForItem(item, false);
    if ( !props )
        return defaultVal;
    std::map<string,bool>::iterator it = props->m_customPropertyMap_bool.find(propertyName);
    if ( it != props->m_customPropertyMap_bool.end() )
        return it->second;
    return defaultVal;
}

//this define saves us writing out 20 functions which are almost exactly the same
#define IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(b2Type, ucName, lcName, ucValType, lcValType)\
    int b2dJson::get##ucName##ByCustom##ucValType( std::string propertyName, lcValType valueToMatch, std::vector<b2Type*>& items )\
    {\
        set<b2Type*>::iterator it = m_##lcName##WithCustomProperties.begin();\
        set<b2Type*>::iterator end = m_##lcName##WithCustomProperties.end();\
        while (it != end)\
        {\
            b2Type* item = *it;\
            if ( hasCustom##ucValType( item, propertyName ) && getCustom##ucValType( item, propertyName ) == valueToMatch )\
                items.push_back( item );\
            ++it;\
        }\
        return items.size();\
    }

IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpBody, Bodies, bodies, Int, int)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpBody, Bodies, bodies, Float, float)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpBody, Bodies, bodies, String, string)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpBody, Bodies, bodies, Vector, cpVect)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpBody, Bodies, bodies, Bool, bool)

IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpShape, Fixtures, fixtures, Int, int)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpShape, Fixtures, fixtures, Float, float)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpShape, Fixtures, fixtures, String, string)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpShape, Fixtures, fixtures, Vector, cpVect)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpShape, Fixtures, fixtures, Bool, bool)

IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpConstraint, Joints, joints, Int, int)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpConstraint, Joints, joints, Float, float)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpConstraint, Joints, joints, String, string)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpConstraint, Joints, joints, Vector, cpVect)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(cpConstraint, Joints, joints, Bool, bool)

IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(b2dJsonImage, Images, images, Int, int)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(b2dJsonImage, Images, images, Float, float)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(b2dJsonImage, Images, images, String, string)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(b2dJsonImage, Images, images, Vector, cpVect)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_VECTOR(b2dJsonImage, Images, images, Bool, bool)

//this define saves us writing out 20 functions which are almost exactly the same
#define IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(b2Type, ucName, lcName, ucValType, lcValType)\
    b2Type* b2dJson::get##ucName##ByCustom##ucValType( std::string propertyName, lcValType valueToMatch )\
    {\
        set<b2Type*>::iterator it = m_##lcName##WithCustomProperties.begin();\
        set<b2Type*>::iterator end = m_##lcName##WithCustomProperties.end();\
        while (it != end)\
        {\
            b2Type* item = *it;\
            if ( hasCustom##ucValType( item, propertyName ) && getCustom##ucValType( item, propertyName ) == valueToMatch )\
                return item;\
            ++it;\
        }\
        return NULL;\
    }

IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpBody, Body, bodies, Int, int)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpBody, Body, bodies, Float, float)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpBody, Body, bodies, String, string)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpBody, Body, bodies, Vector, cpVect)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpBody, Body, bodies, Bool, bool)

IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpShape, Fixture, fixtures, Int, int)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpShape, Fixture, fixtures, Float, float)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpShape, Fixture, fixtures, String, string)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpShape, Fixture, fixtures, Vector, cpVect)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpShape, Fixture, fixtures, Bool, bool)

IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpConstraint, Joint, joints, Int, int)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpConstraint, Joint, joints, Float, float)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpConstraint, Joint, joints, String, string)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpConstraint, Joint, joints, Vector, cpVect)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(cpConstraint, Joint, joints, Bool, bool)

IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(b2dJsonImage, Image, images, Int, int)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(b2dJsonImage, Image, images, Float, float)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(b2dJsonImage, Image, images, String, string)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(b2dJsonImage, Image, images, Vector, cpVect)
IMPLEMENT_GET_BY_CUSTOM_PROPERTY_FUNCTIONS_SINGLE(b2dJsonImage, Image, images, Bool, bool)

Json::Value b2dJson::writeCustomPropertiesToJson(void* item)
{
    Json::Value customPropertiesValue;

    b2dJsonCustomProperties* props = getCustomPropertiesForItem(item, false);
    if ( !props )
        return customPropertiesValue;

    int i = 0;

#define FILL_CUSTOM_PROPERTY_JSON_VALUE(theName,theType)\
    for (std::map<string,theType>::iterator it = props->m_customPropertyMap_##theType.begin(); it != props->m_customPropertyMap_##theType.end(); ++it) {\
        Json::Value propValue;\
        propValue["name"] = it->first;\
        propValue[""#theName] = it->second;\
        customPropertiesValue[i++] = propValue;\
    }

    FILL_CUSTOM_PROPERTY_JSON_VALUE(int,int)
    FILL_CUSTOM_PROPERTY_JSON_VALUE(float,float)
    FILL_CUSTOM_PROPERTY_JSON_VALUE(string,string)
    //FILL_CUSTOM_PROPERTY_JSON_VALUE(vec2,cpVect) handled separately below
    FILL_CUSTOM_PROPERTY_JSON_VALUE(bool,bool)

    for (std::map<string,cpVect>::iterator it = props->m_customPropertyMap_cpVect.begin(); it != props->m_customPropertyMap_cpVect.end(); ++it) {
        Json::Value propValue;
        propValue["name"] = it->first;
        vecToJson("vec2", it->second, propValue);
        customPropertiesValue[i++] = propValue;
    }

    return customPropertiesValue;
}

#define IMPLEMENT_READ_CUSTOM_PROPERTIES_FROM_JSON(b2Type)\
void b2dJson::readCustomPropertiesFromJson(b2Type* item, Json::Value value)\
{\
    if ( ! item )\
        return;\
    if ( ! value.isMember("customProperties") )\
        return;\
\
    int i = 0;\
    Json::Value propValue = value["customProperties"][i++];\
    while ( !propValue.isNull() ) {\
        string propertyName = propValue.get("name", "").asString();\
\
        if ( propValue.isMember("int") ) {\
            int val = propValue.get("int", 0).asInt();\
            setCustomInt(item, propertyName, val);\
        }\
        if ( propValue.isMember("float") ) {\
            float val = propValue.get("float", 0).asFloat();\
            setCustomFloat(item, propertyName, val);\
        }\
        if ( propValue.isMember("string") ) {\
            string val = propValue.get("string", 0).asString();\
            setCustomString(item, propertyName, val);\
        }\
        if ( propValue.isMember("vec2") ) {\
            cpVect val = jsonToVec("vec2", propValue);\
            setCustomVector(item, propertyName, val);\
        }\
        if ( propValue.isMember("bool") ) {\
            bool val = propValue.get("bool", 0).asBool();\
            setCustomBool(item, propertyName, val);\
        }\
\
        propValue = value["customProperties"][i++];\
    }\
}

IMPLEMENT_READ_CUSTOM_PROPERTIES_FROM_JSON(cpBody)
IMPLEMENT_READ_CUSTOM_PROPERTIES_FROM_JSON(cpShape)
IMPLEMENT_READ_CUSTOM_PROPERTIES_FROM_JSON(cpConstraint)
IMPLEMENT_READ_CUSTOM_PROPERTIES_FROM_JSON(b2dJsonImage)
IMPLEMENT_READ_CUSTOM_PROPERTIES_FROM_JSON(cpSpace)

/////////






void b2dJson::floatToJson(const char* name, float f, Json::Value& value)
{
    //cut down on file space for common values
    if ( f == 0 )
        value[name] = 0;
    else if ( f == 1 )
        value[name] = 1;
    else {
        if ( m_useHumanReadableFloats )
            value[name] = f;
        else
            value[name] = floatToHex(f);
    }
}

inline void b2dJson::vecToJson(const char* name, unsigned int v, Json::Value& value, int index)
{
    if (index > -1)
        value[name][index] = v;
    else
        value[name] = v;
}

void b2dJson::vecToJson(const char *name, float v, Json::Value &value, int index)
{
    if (index > -1) {
        if ( m_useHumanReadableFloats ) {
            value[name][index] = v;
        }
        else {
            if ( v == 0 )
                value[name][index] = 0;
            else if ( v == 1 )
                value[name][index] = 1;
            else
                value[name][index] = floatToHex(v);
        }
    }
    else
        floatToJson(name, v, value);
}

inline void b2dJson::vecToJson(const char* name, cpVect vec, Json::Value& value, int index)
{
    if (index > -1) {
        if ( m_useHumanReadableFloats ) {
            value[name]["x"][index] = vec.x;
            value[name]["y"][index] = vec.y;
        }
        else {
            if ( vec.x == 0 )
                value[name]["x"][index] = 0;
            else if ( vec.x == 1 )
                value[name]["x"][index] = 1;
            else
                value[name]["x"][index] = floatToHex(vec.x);
            if ( vec.y == 0 )
                value[name]["y"][index] = 0;
            else if ( vec.y == 1 )
                value[name]["y"][index] = 1;
            else
                value[name]["y"][index] = floatToHex(vec.y);
        }
    }
    else {
        if ( vec.x == 0 && vec.y == 0 )
            value[name] = 0;//cut down on file space for common values
        else {
            floatToJson("x", vec.x, value[name]);
            floatToJson("y", vec.y, value[name]);
        }
    }
}
















void b2dJson::clear()
{
    m_indexToBodyMap.clear();
    m_bodyToIndexMap.clear();
    m_jointToIndexMap.clear();
    m_bodies.clear();
    m_joints.clear();
    m_images.clear();

    m_bodyToNameMap.clear();
    m_fixtureToNameMap.clear();
    m_jointToNameMap.clear();
    m_imageToNameMap.clear();
}

cpSpace *b2dJson::readFromValue(Json::Value worldValue)
{
    clear();

    return j2cpSpace(worldValue);
}

cpSpace* b2dJson::readFromString(std::string str, std::string& errorMsg)
{
    Json::Value worldValue;
    Json::Reader reader;
    if ( ! reader.parse(str, worldValue) )
    {
        //std::cout  << "Failed to parse string\n" << reader.getFormattedErrorMessages();
        errorMsg = string("Failed to parse JSON:\n") + reader.getFormatedErrorMessages();
        return NULL;
    }

    return j2cpSpace(worldValue);
}

cpSpace* b2dJson::readFromFile(const char* filename, std::string& errorMsg)
{
    if (!filename)
        return NULL;

    std::ifstream ifs;
    ifs.open(filename, std::ios::in);
    if (!ifs) {
        //std::cout << "Could not open file " << filename << " for reading\n";
        errorMsg = string("Could not open file '") + string(filename) + string("' for reading");
        return NULL;
    }

    Json::Value worldValue;
    Json::Reader reader;
    if ( ! reader.parse(ifs, worldValue) )
    {
        //std::cout  << "Failed to parse " << filename << std::endl << reader.getFormattedErrorMessages();
        errorMsg = string("Failed to parse '") + string(filename) + string("' : ") + reader.getFormatedErrorMessages();
        ifs.close();
        return NULL;
    }
    ifs.close();

    return j2cpSpace(worldValue);
}

cpSpace* b2dJson::j2cpSpace(Json::Value worldValue)
{
    m_bodies.clear();

    m_simulationPositionIterations = 3;
    m_simulationVelocityIterations = 8;
    m_simulationFPS = 60;

    if ( worldValue.isMember("positionIterations") )
        m_simulationPositionIterations = worldValue["positionIterations"].asInt();
    if ( worldValue.isMember("velocityIterations") )
        m_simulationVelocityIterations = worldValue["velocityIterations"].asInt();
    if ( worldValue.isMember("stepsPerSecond") )
        m_simulationFPS = worldValue["stepsPerSecond"].asFloat();

    float sleepTimeThreshold = 0.5f;
    if ( worldValue.isMember("allowSleep") && ! worldValue["allowSleep"].asBool() )
        sleepTimeThreshold = INFINITY;

    cpSpace *space = cpSpaceNew();
    cpSpaceSetIterations(space, m_simulationVelocityIterations);
    cpSpaceSetGravity(space, jsonToVec("gravity", worldValue));
    cpSpaceSetSleepTimeThreshold(space, sleepTimeThreshold);

    readCustomPropertiesFromJson(space, worldValue);

    int i = 0;
    Json::Value bodyValue = worldValue["body"][i];
    while ( !bodyValue.isNull() ) {
        cpBody* body = j2cpBody(space, bodyValue);
        readCustomPropertiesFromJson(body, bodyValue);
        m_bodies.push_back(body);
        m_indexToBodyMap[i] = body;

        i++;
        bodyValue = worldValue["body"][i];
    }

    //need two passes for joints because gear joints reference other joints
    i = 0;
    Json::Value jointValue = worldValue["joint"][i++];
    while ( !jointValue.isNull() ) {
        if ( jointValue["type"].asString() != "gear" ) {
            cpConstraint* joint = j2cpConstraint(space, jointValue);
            readCustomPropertiesFromJson(joint, jointValue);
            m_joints.push_back(joint);
        }
        jointValue = worldValue["joint"][i++];
    }
/*    i = 0;
    jointValue = worldValue["joint"][i++];
    while ( !jointValue.isNull() ) {
        if ( jointValue["type"].asString() == "gear" ) {
            cpConstraint* joint = j2cpConstraint(space, jointValue);
            readCustomPropertiesFromJson(joint, jointValue);
            m_joints.push_back(joint);
        }
        jointValue = worldValue["joint"][i++];
    }
*/
    i = 0;
    Json::Value imageValue = worldValue["image"][i++];
    while ( !imageValue.isNull() ) {
        b2dJsonImage* img = j2b2dJsonImage(imageValue);
        readCustomPropertiesFromJson(img, imageValue);
        m_images.push_back(img);
        addImage(img);

        imageValue = worldValue["image"][i++];
    }

    return space;
}

cpBody* b2dJson::j2cpBody(cpSpace* space, Json::Value bodyValue)
{
    cpBody* body = NULL;

    /* unused settings
    bodyDef.linearDamping = jsonToFloat("linearDamping", bodyValue, -1, 0);
    bodyDef.angularDamping = jsonToFloat("angularDamping", bodyValue, -1, 0);
    bodyDef.gravityScale = jsonToFloat("gravityScale", bodyValue, -1, 1);
    bodyDef.allowSleep = bodyValue.get("allowSleep",true).asBool();
    bodyDef.bullet = bodyValue.get("bullet",false).asBool();
    bodyDef.active = bodyValue.get("active",true).asBool();
    */

    int type = bodyValue["type"].asInt();
    if ( type == 2 ) {//dynamic
        body = cpBodyNew(1, 1);
        cpSpaceAddBody(space, body);
    }
    else
        body = cpBodyNewStatic();
    cpBodySetPos(body, jsonToVec("position", bodyValue));
    cpBodySetAngle(body, jsonToFloat("angle", bodyValue ));
    cpBodySetVel(body, jsonToVec("linearVelocity", bodyValue));
    cpBodySetAngVel(body, jsonToFloat("angularVelocity", bodyValue));
    if ( ! bodyValue.get("awake", false).asBool() )
        cpBodySleep(body);
    if ( bodyValue.get("fixedRotation",false).asBool() )
        cpBodySetMoment(body, INFINITY);

    string bodyName = bodyValue.get("name","").asString();
    if ( bodyName != "" ) {
        //printf("Found named body: %s\n",bodyName.c_str());
        setBodyName(body, bodyName.c_str());
    }

    int i = 0;
    Json::Value fixtureValue = bodyValue["fixture"][i++];
    while ( !fixtureValue.isNull() ) {
        cpShape* fixture = j2cpShape(space, body, fixtureValue);
        readCustomPropertiesFromJson(fixture, fixtureValue);

        fixtureValue = bodyValue["fixture"][i++];
    }

    /*
    //may be necessary if user has overridden mass characteristics
    b2MassData massData;
    massData.mass = jsonToFloat("massData-mass", bodyValue);
    massData.center = jsonToVec("massData-center", bodyValue);
    massData.I = jsonToFloat("massData-I", bodyValue);
    body->SetMassData(&massData);
    */

    return body;
}

cpShape* b2dJson::j2cpShape(cpSpace* space, cpBody* body, Json::Value fixtureValue)
{
    if ( fixtureValue.isNull() )
        return NULL;

    cpShape* shape = NULL;
    vector<cpShape> allShapes;//used to apply friction etc to all created shapes at the end of the function (for chain shapes)

    float density = jsonToFloat("density", fixtureValue);

    if ( !fixtureValue["circle"].isNull() ) {
        cpVect center = jsonToVec("center", fixtureValue["circle"]);
        float radius = jsonToFloat("radius", fixtureValue["circle"]);
        shape = cpCircleShapeNew(body, radius, center);
        float area = cpAreaForCircle(0, radius);
        float mass = density * area;
        cpBodySetMoment( body, cpMomentForCircle(mass, 0, radius, center) );
    }
    else if ( !fixtureValue["edge"].isNull() ) {
        cpVect vertex1 = jsonToVec("vertex1", fixtureValue["edge"]);
        cpVect vertex2 = jsonToVec("vertex2", fixtureValue["edge"]);
        float radius = 0;
        shape = cpSegmentShapeNew(body, vertex1, vertex2, radius);
        float area = cpAreaForSegment(vertex1, vertex2, radius);
        float mass = density * area;
        if ( ! cpBodyIsStatic(body) )
            cpBodySetMoment( body, cpMomentForSegment(mass, vertex1, vertex2) );
    }
    /*else if ( !fixtureValue["loop"].isNull() ) { //support old format (r197)
        b2ChainShape chainShape;
        int numVertices = fixtureValue["loop"]["vertices"]["x"].size();
        cpVect* vertices = new cpVect[numVertices];
        for (int i = 0; i < numVertices; i++)
            vertices[i] = jsonToVec("vertices", fixtureValue["loop"], i);
        chainShape.CreateLoop(vertices, numVertices);
        fixtureDef.shape = &chainShape;
        shape = body->CreateFixture(&fixtureDef);
        delete[] vertices;
    }*/
    else if ( !fixtureValue["chain"].isNull() ) {
        int numVertices = fixtureValue["chain"]["vertices"]["x"].size();
        cpVect* vertices = new cpVect[numVertices];
        for (int i = 0; i < numVertices; i++)
            vertices[i] = jsonToVec("vertices", fixtureValue["chain"], i);
        float radius = 0;
        for (int i = 0; i < numVertices; i++) {
            cpVect vertex1 = vertices[i];
            cpVect vertex2 = vertices[(i+1) % numVertices];
            shape = cpSegmentShapeNew(body, vertex1, vertex2, radius);//this function will end up only returning the last shape
            float area = cpAreaForSegment(vertex1, vertex2, radius);
            float mass = density * area;
            if ( ! cpBodyIsStatic(body) )
                cpBodySetMoment( body, cpMomentForSegment(mass, vertex1, vertex2) );//hmm. How to set correct moment without clobbering moment from existing shapes?
        }
        delete[] vertices;
    }
    else if ( !fixtureValue["polygon"].isNull() ) {
        int numVertices = fixtureValue["polygon"]["vertices"]["x"].size();
        cpVect vertices[b2_maxPolygonVertices];
        int k = 0;
        for (int i = numVertices-1; i >= 0; i--) // ohh... clockwise?!
            vertices[k++] = jsonToVec("vertices", fixtureValue["polygon"], i);
        shape = cpPolyShapeNew(body, numVertices, vertices, cpvzero);
        float area = cpAreaForPoly(numVertices, vertices);
        float mass = density * area;
        if ( ! cpBodyIsStatic(body) )
            cpBodySetMoment( body, cpMomentForPoly(mass, numVertices, vertices, cpvzero) );
    }

    string fixtureName = fixtureValue.get("name","").asString();
    if ( fixtureName != "" ) {
        setFixtureName(shape, fixtureName.c_str());
    }

    if ( shape ) {
        cpSpaceAddShape( space, shape );
        cpShapeSetElasticity(shape, jsonToFloat("restitution", fixtureValue));
        cpShapeSetFriction(shape, jsonToFloat("friction", fixtureValue));
        cpShapeSetSensor(shape, fixtureValue.get("sensor",false).asBool());

        cpShapeSetLayers(shape, fixtureValue.get("filter-maskBits",0xffff).asInt());

        /* unused settings
        fixtureDef.filter.categoryBits = fixtureValue.get("filter-categoryBits",0x0001).asInt();
        fixtureDef.filter.maskBits = fixtureValue.get("filter-maskBits",0xffff).asInt();
        fixtureDef.filter.groupIndex = fixtureValue.get("filter-groupIndex",0).asInt();
        */
    }

    return shape;
}

cpConstraint* b2dJson::j2cpConstraint(cpSpace* space, Json::Value jointValue)
{
    cpConstraint* joint = NULL;

    int bodyIndexA = jointValue["bodyA"].asInt();
    int bodyIndexB = jointValue["bodyB"].asInt();
    if ( bodyIndexA >= (int)m_bodies.size() || bodyIndexB >= (int)m_bodies.size() )
        return NULL;
/*
    //keep these in scope after the if/else below
    b2RevoluteJointDef revoluteDef;
    b2PrismaticJointDef prismaticDef;
    b2DistanceJointDef distanceDef;
    b2PulleyJointDef pulleyDef;
    b2MouseJointDef mouseDef;
    b2GearJointDef gearDef;
    b2WheelJointDef wheelDef;
    b2WeldJointDef weldDef;
    b2FrictionJointDef frictionDef;
    b2RopeJointDef ropeDef;

    //will be used to select one of the above to work with
    cpConstraintDef* jointDef = NULL;
*/
    //cpVect mouseJointTarget;
    std::string type = jointValue["type"].asString();    
    if ( type == "revolute" )
    {
        cpVect localAnchorA = jsonToVec("anchorA", jointValue);
        cpVect localAnchorB = jsonToVec("anchorB", jointValue);
        joint = cpPivotJointNew2(m_bodies[bodyIndexA], m_bodies[bodyIndexB], localAnchorA, localAnchorB);
        cpSpaceAddConstraint(space, joint);
        /* unused settings
        revoluteDef.referenceAngle = jsonToFloat("refAngle", jointValue);
        revoluteDef.enableLimit = jointValue["enableLimit"].asBool();
        revoluteDef.lowerAngle = jsonToFloat("lowerLimit", jointValue);
        revoluteDef.upperAngle = jsonToFloat("upperLimit", jointValue);
        revoluteDef.enableMotor = jointValue["enableMotor"].asBool();
        revoluteDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
        revoluteDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
        */
    }
    /*else if ( type == "prismatic" )
    {
        jointDef = &prismaticDef;
        prismaticDef.localAnchorA = jsonToVec("anchorA", jointValue);
        prismaticDef.localAnchorB = jsonToVec("anchorB", jointValue);
        if ( !jointValue["localAxisA"].isNull() )
            prismaticDef.localAxisA = jsonToVec("localAxisA", jointValue);
        else
            prismaticDef.localAxisA = jsonToVec("localAxis1", jointValue);
        prismaticDef.referenceAngle = jsonToFloat("refAngle", jointValue);
        prismaticDef.enableLimit = jointValue["enableLimit"].asBool();
        prismaticDef.lowerTranslation = jsonToFloat("lowerLimit", jointValue);
        prismaticDef.upperTranslation = jsonToFloat("upperLimit", jointValue);
        prismaticDef.enableMotor = jointValue["enableMotor"].asBool();
        prismaticDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
        prismaticDef.maxMotorForce = jsonToFloat("maxMotorForce", jointValue);
    }*/
    else if ( type == "distance" )
    {
        cpVect localAnchorA = jsonToVec("anchorA", jointValue);
        cpVect localAnchorB = jsonToVec("anchorB", jointValue);
        float length = jsonToFloat("length", jointValue);
        float stiffness = jsonToFloat("frequency", jointValue);
        float damping = jsonToFloat("dampingRatio", jointValue);
        joint = cpDampedSpringNew(m_bodies[bodyIndexA], m_bodies[bodyIndexB], localAnchorA, localAnchorB, length, stiffness, damping);
        cpSpaceAddConstraint(space, joint);
    }
    /*else if ( type == "pulley" )
    {
        jointDef = &pulleyDef;
        pulleyDef.groundAnchorA = jsonToVec("groundAnchorA", jointValue);
        pulleyDef.groundAnchorB = jsonToVec("groundAnchorB", jointValue);
        pulleyDef.localAnchorA = jsonToVec("anchorA", jointValue);
        pulleyDef.localAnchorB = jsonToVec("anchorB", jointValue);
        pulleyDef.lengthA = jsonToFloat("lengthA", jointValue);
        pulleyDef.lengthB = jsonToFloat("lengthB", jointValue);
        pulleyDef.ratio = jsonToFloat("ratio", jointValue);
    }
    else if ( type == "mouse" )
    {
        jointDef = &mouseDef;
        mouseJointTarget = jsonToVec("target", jointValue);
        mouseDef.target = jsonToVec("anchorB", jointValue);//alter after creating joint
        mouseDef.maxForce = jsonToFloat("maxForce", jointValue);
        mouseDef.frequencyHz = jsonToFloat("frequency", jointValue);
        mouseDef.dampingRatio = jsonToFloat("dampingRatio", jointValue);
    }
    else if ( type == "gear" )
    {
        jointDef = &gearDef;
        int jointIndex1 = jointValue["joint1"].asInt();
        int jointIndex2 = jointValue["joint2"].asInt();
        gearDef.joint1 = m_joints[jointIndex1];
        gearDef.joint2 = m_joints[jointIndex2];
        gearDef.ratio = jsonToFloat("ratio", jointValue);
    }*/
    else if ( type == "wheel" )
    {
        cpVect localAnchorA = jsonToVec("anchorA", jointValue);
        cpVect localAnchorB = jsonToVec("anchorB", jointValue);
        cpVect localAxisA = jsonToVec("localAxisA", jointValue);
        joint = cpGrooveJointNew(m_bodies[bodyIndexA], m_bodies[bodyIndexB], localAnchorA, cpvadd(localAnchorA,localAxisA), localAnchorB);
        cpSpaceAddConstraint(space, joint);

        //also add a distance joint
        //Chipmunk groove joints have limits whereas b2WheelJoints do not, and chipmunk damped springs
        //seem to be only one way (ie. only preventing compression) whereas b2WheelJoints are two-way,
        //for lack of a better description. This can be accounted for in a rather messy way by using
        //the length of the axis to adjust the target position of the spring so that it is not right
        //on top of the limit of the groove joint.
        float length = cpvlength(localAxisA)*0.5f;
        float stiffness = jsonToFloat("springFrequency", jointValue);
        float damping = jsonToFloat("springDampingRatio", jointValue);
        joint = cpDampedSpringNew(m_bodies[bodyIndexA], m_bodies[bodyIndexB], localAnchorA, localAnchorB, length, stiffness, damping);
        cpSpaceAddConstraint(space, joint);

        /* unused settings
        wheelDef.localAnchorA = jsonToVec("anchorA", jointValue);
        wheelDef.localAnchorB = jsonToVec("anchorB", jointValue);
        wheelDef.localAxisA = jsonToVec("localAxisA", jointValue);
        wheelDef.enableMotor = jointValue["enableMotor"].asBool();
        wheelDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
        wheelDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
        wheelDef.frequencyHz = jsonToFloat("springFrequency", jointValue);
        wheelDef.dampingRatio = jsonToFloat("springDampingRatio", jointValue);
        */
    }
    /*else if ( type == "weld" )
    {
        jointDef = &weldDef;
        weldDef.localAnchorA = jsonToVec("anchorA", jointValue);
        weldDef.localAnchorB = jsonToVec("anchorB", jointValue);
        weldDef.referenceAngle = 0;//jsonToFloat("refAngle", jointValue);
    }
    else if ( type == "friction" )
    {
        jointDef = &frictionDef;
        frictionDef.localAnchorA = jsonToVec("anchorA", jointValue);
        frictionDef.localAnchorB = jsonToVec("anchorB", jointValue);
        frictionDef.maxForce = jsonToFloat("maxForce", jointValue);
        frictionDef.maxTorque = jsonToFloat("maxTorque", jointValue);
    }
    else if ( type == "rope" )
    {
        jointDef = &ropeDef;
        ropeDef.localAnchorA = jsonToVec("anchorA", jointValue);
        ropeDef.localAnchorB = jsonToVec("anchorB", jointValue);
        ropeDef.maxLength = jsonToFloat("maxLength", jointValue);
    }*/

    /*if ( jointDef ) {
        //set features common to all joints
        jointDef->bodyA = m_bodies[bodyIndexA];
        jointDef->bodyB = m_bodies[bodyIndexB];
        jointDef->collideConnected = jointValue.get("collideConnected",false).asBool();

        joint = world->CreateJoint(jointDef);

        if ( type == "mouse" )
            ((b2MouseJoint*)joint)->SetTarget(mouseJointTarget);

        string jointName = jointValue.get("name","").asString();
        if ( jointName != "" ) {
            setJointName(joint, jointName.c_str());
        }
    }*/

    return joint;
}

b2dJsonImage* b2dJson::j2b2dJsonImage(Json::Value imageValue)
{
    b2dJsonImage* img = new b2dJsonImage();

    if ( imageValue["body"].isInt() )
        img->body = lookupBodyFromIndex( imageValue["body"].asInt() );

    if ( imageValue["name"].isString() )
        img->name = imageValue["name"].asString();

    if ( imageValue["file"].isString() )
        img->file = imageValue["file"].asString();

    img->center = jsonToVec("center", imageValue);
    img->angle = jsonToFloat("angle", imageValue);
    img->scale = jsonToFloat("scale", imageValue);
    img->opacity = jsonToFloat("opacity", imageValue);
    img->renderOrder = jsonToFloat("renderOrder", imageValue);

    if ( imageValue["flip"].isBool() )
        img->flip = imageValue["flip"].asBool();

    if ( imageValue["filter"].isInt() )
        img->filter = imageValue["filter"].asInt();

    for (int i = 0; i < 4; i++)
        img->m_corners[i] = jsonToVec("corners", imageValue, i);

    if ( imageValue["glVertexPointer"].isArray() && imageValue["glTexCoordPointer"].isArray() &&
         (imageValue["glVertexPointer"].size()  ==  imageValue["glTexCoordPointer"].size()) ) {
        int numFloats = imageValue["glVertexPointer"].size();
        img->numPoints = numFloats / 2;
        img->points = new float[numFloats];
        img->uvCoords = new float[numFloats];
        for (int i = 0; i < numFloats; i++) {
            img->points[i] = jsonToFloat("glVertexPointer", imageValue, i);
            img->uvCoords[i] = jsonToFloat("glTexCoordPointer", imageValue, i);
        }
    }

    if ( imageValue["glDrawElements"].isArray() ) {
        img->numIndices = imageValue["glDrawElements"].size();
        img->indices = new unsigned short[img->numIndices];
        for (int i = 0; i < img->numIndices; i++)
            img->indices[i] = imageValue["glDrawElements"][i].asInt();
    }

    return img;
}

float b2dJson::jsonToFloat(const char* name, Json::Value& value, int index, float defaultValue)
{
    if ( ! value.isMember(name) )
        return defaultValue;

    if ( index > -1 ) {
        if ( value[name][index].isNull() )
            return defaultValue;
        else if ( value[name][index].isInt() )
            return value[name][index].asInt();//usually 0 or 1
        else if ( value[name][index].isString() )
            return hexToFloat( value[name][index].asString() );
        else
            return value[name][index].asFloat();
    }
    else {
        if ( value[name].isNull() )
            return defaultValue;
        else if ( value[name].isInt() )
            return value[name].asInt();//usually 0 or 1
        else if ( value[name].isString() )
            return hexToFloat( value[name].asString() );
        else
            return value[name].asFloat();
    }
}

cpVect b2dJson::jsonToVec(const char* name, Json::Value& value, int index, cpVect defaultValue)
{
    cpVect vec = defaultValue;

    if ( ! value.isMember(name) )
        return defaultValue;

    if (index > -1) {

        if ( value[name]["x"][index].isInt() ) //usually 0 or 1
            vec.x = value[name]["x"][index].asInt();
        else if ( value[name]["x"][index].isString() )
            vec.x = hexToFloat(value[name]["x"][index].asString());
        else
            vec.x = value[name]["x"][index].asFloat();

        if ( value[name]["y"][index].isInt() ) //usually 0 or 1
            vec.y = value[name]["y"][index].asInt();
        else if ( value[name]["y"][index].isString() )
            vec.y = hexToFloat(value[name]["y"][index].asString());
        else
            vec.y = value[name]["y"][index].asFloat();
    }
    else {

        if ( value[name].isInt() ) //zero vector
            vec = cpvzero;
        else {
            vec.x = jsonToFloat("x", value[name]);
            vec.y = jsonToFloat("y", value[name]);
        }
    }

    return vec;
}



















std::string b2dJson::floatToHex(float f)
{
    char buf[16];
    //sprintf(buf, "%08X", *((int*)(&f)) ); dereferencing type-punned pointer will break strict-aliasing rules
    int* i = (int*)(&f);
    sprintf(buf, "%08X", *i );
    return std::string(buf);
}

float b2dJson::hexToFloat(std::string str)
{
    int strLen = 8;//32 bit float
    unsigned char bytes[4];
    int bptr = (strLen / 2) - 1;

    for (int i = 0; i < strLen; i++){
        unsigned char   c;
        c = str[i];
        if (c > '9') c -= 7;
        c <<= 4;
        bytes[bptr] = c;

        ++i;
        c = str[i];
        if (c > '9') c -= 7;
        c -= '0';
        bytes[bptr] |= c;

        --bptr;
    }

    //return *((float*)bytes); dereferencing type-punned pointer will break strict-aliasing rules
    float* f = (float*)bytes;
    return *f;
}

cpBody* b2dJson::lookupBodyFromIndex( unsigned int index )
{
    std::map<int,cpBody*>::iterator it = m_indexToBodyMap.find(index);
    if ( it != m_indexToBodyMap.end() )
        return it->second;
    else
        return NULL;
}

int b2dJson::lookupBodyIndex( cpBody* body )
{
    std::map<cpBody*,int>::iterator it = m_bodyToIndexMap.find(body);
    if ( it != m_bodyToIndexMap.end() )
        return it->second;
    else
        return -1;
}

int b2dJson::lookupJointIndex( cpConstraint* joint )
{
    std::map<cpConstraint*,int>::iterator it = m_jointToIndexMap.find(joint);
    if ( it != m_jointToIndexMap.end() )
        return it->second;
    else
        return -1;
}




string b2dJson::getBodyName(cpBody* body)
{
    map<cpBody*,string>::iterator it = m_bodyToNameMap.find( body );
    if ( it == m_bodyToNameMap.end() )
        return "";
    return it->second;
}

string b2dJson::getFixtureName(cpShape* fixture)
{
    map<cpShape*,string>::iterator it = m_fixtureToNameMap.find( fixture );
    if ( it == m_fixtureToNameMap.end() )
        return "";
    return it->second;
}

string b2dJson::getJointName(cpConstraint* joint)
{
    map<cpConstraint*,string>::iterator it = m_jointToNameMap.find( joint );
    if ( it == m_jointToNameMap.end() )
        return "";
    return it->second;
}

string b2dJson::getImageName(b2dJsonImage *img)
{
    map<b2dJsonImage*,string>::iterator it = m_imageToNameMap.find( img );
    if ( it == m_imageToNameMap.end() )
        return "";
    return it->second;
}

int b2dJson::getBodiesByName(string name, vector<cpBody*>& bodies)
{
    map<cpBody*,string>::iterator it = m_bodyToNameMap.begin();
    map<cpBody*,string>::iterator end = m_bodyToNameMap.end();
    while (it != end) {
        if ( it->second == name )
            bodies.push_back(it->first);
        ++it;
    }
    return bodies.size();
}

int b2dJson::getFixturesByName(string name, vector<cpShape*>& fixtures)
{
    map<cpShape*,string>::iterator it = m_fixtureToNameMap.begin();
    map<cpShape*,string>::iterator end = m_fixtureToNameMap.end();
    while (it != end) {
        if ( it->second == name )
            fixtures.push_back(it->first);
        ++it;
    }
    return fixtures.size();
}

int b2dJson::getJointsByName(string name, vector<cpConstraint*>& joints)
{
    map<cpConstraint*,string>::iterator it = m_jointToNameMap.begin();
    map<cpConstraint*,string>::iterator end = m_jointToNameMap.end();
    while (it != end) {
        if ( it->second == name )
            joints.push_back(it->first);
        ++it;
    }
    return joints.size();
}

int b2dJson::getImagesByName(string name, vector<b2dJsonImage*> &images)
{
    map<b2dJsonImage*,string>::iterator it = m_imageToNameMap.begin();
    map<b2dJsonImage*,string>::iterator end = m_imageToNameMap.end();
    while (it != end) {
        if ( it->second == name )
            images.push_back(it->first);
        ++it;
    }
    return images.size();
}

int b2dJson::getAllImages(vector<b2dJsonImage*> &images)
{
    images.insert( images.begin(), m_images.begin(), m_images.end() );
    std::sort(images.begin(), images.end(), b2dJsonImage_renderOrder_ascending);
    return images.size();
}


cpBody* b2dJson::getBodyByName(string name)
{
    map<cpBody*,string>::iterator it = m_bodyToNameMap.begin();
    map<cpBody*,string>::iterator end = m_bodyToNameMap.end();
    while (it != end) {
        if ( it->second == name )
            return it->first;
        ++it;
    }
    return NULL;
}

cpShape* b2dJson::getFixtureByName(string name)
{
    map<cpShape*,string>::iterator it = m_fixtureToNameMap.begin();
    map<cpShape*,string>::iterator end = m_fixtureToNameMap.end();
    while (it != end) {
        if ( it->second == name )
            return it->first;
        ++it;
    }
    return NULL;
}

cpConstraint* b2dJson::getJointByName(string name)
{
    map<cpConstraint*,string>::iterator it = m_jointToNameMap.begin();
    map<cpConstraint*,string>::iterator end = m_jointToNameMap.end();
    while (it != end) {
        if ( it->second == name )
            return it->first;
        ++it;
    }
    return NULL;
}



