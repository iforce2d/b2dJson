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

#ifndef B2DJSON_H
#define B2DJSON_H

#include <stdio.h>
#include <map>
#include <set>
#include <string>
#include <Box2D/Box2D.h>
#include "json/json.h"

class b2dJsonImage;

class b2dJsonColor4 {
public:
    int r, g, b, a;
    b2dJsonColor4() { r = g = b = a = 255; }
};

class b2dJsonCustomProperties {
public:
    std::map<std::string, int> m_customPropertyMap_int;
    std::map<std::string, float> m_customPropertyMap_float;
    std::map<std::string, std::string> m_customPropertyMap_string;
    std::map<std::string, b2Vec2> m_customPropertyMap_b2Vec2;
    std::map<std::string, bool> m_customPropertyMap_bool;
    std::map<std::string, b2dJsonColor4> m_customPropertyMap_color;
};

class b2dJson
{
protected:
    bool m_useHumanReadableFloats;
    std::map<int,b2Body*> m_indexToBodyMap;
    std::map<b2Body*,int> m_bodyToIndexMap;
    std::map<b2Joint*,int> m_jointToIndexMap;
    std::vector<b2Body*> m_bodies;
    std::vector<b2Joint*> m_joints;
    std::vector<b2dJsonImage*> m_images;

    std::map<b2Body*,std::string> m_bodyToNameMap;
    std::map<b2Fixture*,std::string> m_fixtureToNameMap;
    std::map<b2Joint*,std::string> m_jointToNameMap;
    std::map<b2dJsonImage*,std::string> m_imageToNameMap;

    std::map<b2Body*,std::string> m_bodyToPathMap;
    std::map<b2Fixture*,std::string> m_fixtureToPathMap;
    std::map<b2Joint*,std::string> m_jointToPathMap;
    std::map<b2dJsonImage*,std::string> m_imageToPathMap;

    // This maps an item (b2Body*, b2Fixture* etc) to a set of custom properties.
    // Use NULL for world properties.
    std::map<void*,b2dJsonCustomProperties*> m_customPropertiesMap;

    // These are necessary to know what type of item the entries in the map above
    // are, which is necessary for the getBodyByCustomInt type functions.
    // We could have used a separate map for each item type, but there are many
    // combinations of item type and property type and the overall amount of
    // explicit coding to do becomes very large for no real benefit.
    std::set<b2Body*> m_bodiesWithCustomProperties;
    std::set<b2Fixture*> m_fixturesWithCustomProperties;
    std::set<b2Joint*> m_jointsWithCustomProperties;
    std::set<b2dJsonImage*> m_imagesWithCustomProperties;
    std::set<b2World*> m_worldsWithCustomProperties;

public:
    //constructor
    b2dJson(bool useHumanReadableFloats = false);
    ~b2dJson();

    void clear();

    //writing functions
    Json::Value writeToValue(b2World* world);
    std::string writeToString(b2World* world);
    bool writeToFile(b2World* world, const char* filename);

    Json::Value b2j(b2World* world);
    Json::Value b2j(b2Body* body);
    Json::Value b2j(b2Fixture* fixture);
    Json::Value b2j(b2Joint* joint);
    Json::Value b2j(b2dJsonImage* image);

    void setBodyName(b2Body* body, const char* name);
    void setFixtureName(b2Fixture* fixture, const char* name);
    void setJointName(b2Joint* joint, const char* name);
    void setImageName(b2dJsonImage* image, const char* name);

    void setBodyPath(b2Body* body, const char* path);
    void setFixturePath(b2Fixture* fixture, const char* path);
    void setJointPath(b2Joint* joint, const char* path);
    void setImagePath(b2dJsonImage* image, const char* path);

    void addImage(b2dJsonImage* image);

    //reading functions
    b2World* readFromValue(Json::Value worldValue, b2World *existingWorld = NULL);
    b2World* readFromString(std::string str, std::string& errorMsg, b2World *existingWorld = NULL);
    b2World* readFromFile(const char* filename, std::string& errorMsg, b2World* existingWorld = NULL);

    //backward compatibility
    bool readIntoWorldFromValue(b2World *existingWorld, Json::Value &worldValue)                    { return readFromValue(worldValue, existingWorld); }
    bool readIntoWorldFromString(b2World *existingWorld, std::string str, std::string& errorMsg)    { return readFromString(str, errorMsg, existingWorld); }
    bool readIntoWorldFromFile(b2World *existingWorld, const char* filename, std::string& errorMsg) { return readFromFile(filename, errorMsg, existingWorld); }

    b2World* j2b2World(Json::Value &worldValue, b2World* world = NULL);
    b2Body* j2b2Body(b2World* world, Json::Value& bodyValue);
    b2Fixture* j2b2Fixture(b2Body* body, Json::Value& fixtureValue);
    b2Joint* j2b2Joint(b2World* world, Json::Value& jointValue);
    b2dJsonImage* j2b2dJsonImage(Json::Value& imageValue);
    
    //function copies json world into existing world
    bool j2Intob2World(b2World *world, Json::Value& worldValue);

    int getBodiesByName(std::string name, std::vector<b2Body*>& bodies);
    int getFixturesByName(std::string name, std::vector<b2Fixture*>& fixtures);
    int getJointsByName(std::string name, std::vector<b2Joint*>& joints);
    int getImagesByName(std::string name, std::vector<b2dJsonImage*>& images);

    int getBodiesByPath(std::string path, std::vector<b2Body*>& bodies);
    int getFixturesByPath(std::string path, std::vector<b2Fixture*>& fixtures);
    int getJointsByPath(std::string path, std::vector<b2Joint*>& joints);
    int getImagesByPath(std::string path, std::vector<b2dJsonImage*>& images);

    int getAllBodies(std::vector<b2Body*>& bodies);
    int getAllFixtures(std::vector<b2Fixture*>& fixtures);
    int getAllJoints(std::vector<b2Joint*>& joints);
    int getAllImages(std::vector<b2dJsonImage*>& images);

    b2Body* getBodyByName(std::string name);
    b2Fixture* getFixtureByName(std::string name);
    b2Joint* getJointByName(std::string name);
    b2dJsonImage* getImageByName(std::string name);

    b2Body* getBodyByPathAndName(std::string path, std::string name);
    b2Fixture* getFixtureByPathAndName(std::string path, std::string name);
    b2Joint* getJointByPathAndName(std::string path, std::string name);
    b2dJsonImage* getImageByPathAndName(std::string path, std::string name);

    std::map<b2Joint*,std::string> getJointToNameMap() const { return m_jointToNameMap; }
    std::map<b2Fixture*,std::string> getFixtureToNameMap() const { return m_fixtureToNameMap; }

    std::string getBodyName(b2Body* body);
    std::string getFixtureName(b2Fixture* fixture);
    std::string getJointName(b2Joint* joint);
    std::string getImageName(b2dJsonImage* img);

    std::string getBodyPath(b2Body* body);
    std::string getFixturePath(b2Fixture* fixture);
    std::string getJointPath(b2Joint* joint);
    std::string getImagePath(b2dJsonImage* img);

    ////// custom properties

    b2dJsonCustomProperties* getCustomPropertiesForItem(void* item, bool createIfNotExisting);
protected:
    void setCustomInt(void* item, std::string propertyName, int val);
    void setCustomFloat(void* item, std::string propertyName, float val);
    void setCustomString(void* item, std::string propertyName, std::string val);
    void setCustomVector(void* item, std::string propertyName, b2Vec2 val);
    void setCustomBool(void* item, std::string propertyName, bool val);
    void setCustomColor(void* item, std::string propertyName, b2dJsonColor4 val);

public:
//this define saves us writing out 25 functions which are almost exactly the same
#define DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(ucType, lcType)\
    void setCustom##ucType(b2Body* item, std::string propertyName, lcType val)          { m_bodiesWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }\
    void setCustom##ucType(b2Fixture* item, std::string propertyName, lcType val)       { m_fixturesWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }\
    void setCustom##ucType(b2Joint* item, std::string propertyName, lcType val)         { m_jointsWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }\
    void setCustom##ucType(b2dJsonImage* item, std::string propertyName, lcType val)    { m_imagesWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }\
    void setCustom##ucType(b2World* item, std::string propertyName, lcType val)         { m_worldsWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }

    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Int, int)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Float, float)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(String, std::string)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Vector, b2Vec2)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Bool, bool)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Color, b2dJsonColor4)

    bool hasCustomInt(void* item, std::string propertyName);
    bool hasCustomFloat(void* item, std::string propertyName);
    bool hasCustomString(void* item, std::string propertyName);
    bool hasCustomVector(void* item, std::string propertyName);
    bool hasCustomBool(void* item, std::string propertyName);
    bool hasCustomColor(void* item, std::string propertyName);

    int getCustomInt(void* item, std::string propertyName, int defaultVal = 0);
    float getCustomFloat(void* item, std::string propertyName, float defaultVal = 0);
    std::string getCustomString(void* item, std::string propertyName, std::string defaultVal = "");
    b2Vec2 getCustomVector(void* item, std::string propertyName, b2Vec2 defaultVal = b2Vec2(0,0));
    bool getCustomBool(void* item, std::string propertyName, bool defaultVal = false);
    b2dJsonColor4 getCustomColor(void* item, std::string propertyName, b2dJsonColor4 defaultVal = b2dJsonColor4());

//this define saves us writing out 20 functions which are almost exactly the same
#define DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(ucType, lcType)\
    int getBodiesByCustom##ucType(   std::string propertyName, lcType valueToMatch, std::vector<b2Body*>& bodies);\
    int getFixturesByCustom##ucType( std::string propertyName, lcType valueToMatch, std::vector<b2Fixture*>& fixtures);\
    int getJointsByCustom##ucType(   std::string propertyName, lcType valueToMatch, std::vector<b2Joint*>& joints);\
    int getImagesByCustom##ucType(   std::string propertyName, lcType valueToMatch, std::vector<b2dJsonImage*>& images);

    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(Int, int)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(Float, float)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(String, std::string)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(Vector, b2Vec2)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(Bool, bool)

//this define saves us writing out 20 functions which are almost exactly the same
#define DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(ucType, lcType)\
    b2Body*         getBodyByCustom##ucType(    std::string propertyName, lcType valueToMatch);\
    b2Fixture*      getFixtureByCustom##ucType( std::string propertyName, lcType valueToMatch);\
    b2Joint*        getJointByCustom##ucType(   std::string propertyName, lcType valueToMatch);\
    b2dJsonImage*   getImageByCustom##ucType(   std::string propertyName, lcType valueToMatch);

    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(Int, int)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(Float, float)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(String, std::string)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(Vector, b2Vec2)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(Bool, bool)

    //////



protected:
    //member helpers
    void vecToJson(const char* name, unsigned int v, Json::Value& value, int index = -1);
    void vecToJson(const char* name, float v, Json::Value& value, int index = -1);
    void vecToJson(const char* name, b2Vec2 vec, Json::Value& value, int index = -1);
    void floatToJson(const char* name, float f, Json::Value& value);
    b2Body* lookupBodyFromIndex( unsigned int index );
    int lookupBodyIndex( b2Body* body );
    int lookupJointIndex( b2Joint* joint );

    Json::Value writeCustomPropertiesToJson(void* item);
    void readCustomPropertiesFromJson(b2Body* item, Json::Value value);
    void readCustomPropertiesFromJson(b2Fixture* item, Json::Value value);
    void readCustomPropertiesFromJson(b2Joint* item, Json::Value value);
    void readCustomPropertiesFromJson(b2dJsonImage* item, Json::Value value);
    void readCustomPropertiesFromJson(b2World* item, Json::Value value);

public:
    //static helpers
    static std::string floatToHex(float f);
    static float hexToFloat(std::string str);
    static float jsonToFloat(const char* name, Json::Value& value, int index = -1, float defaultValue = 0);
    static b2Vec2 jsonToVec(const char* name, Json::Value& value, int index = -1, b2Vec2 defaultValue = b2Vec2(0,0));
};

#endif // B2DJSON_H







