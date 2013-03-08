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
#include "json/json.h"

#include "chipmunk.h"

class b2dJsonImage;

class b2dJsonCustomProperties {
public:
    std::map<std::string, int> m_customPropertyMap_int;
    std::map<std::string, float> m_customPropertyMap_float;
    std::map<std::string, std::string> m_customPropertyMap_string;
    std::map<std::string, cpVect> m_customPropertyMap_cpVect;
    std::map<std::string, bool> m_customPropertyMap_bool;
};

class b2dJson
{
protected:
    bool m_useHumanReadableFloats;

    int m_simulationPositionIterations;
    int m_simulationVelocityIterations;
    float m_simulationFPS;

    std::map<int,cpBody*> m_indexToBodyMap;
    std::map<cpBody*,int> m_bodyToIndexMap;
    std::map<cpConstraint*,int> m_jointToIndexMap;
    std::vector<cpBody*> m_bodies;
    std::vector<cpConstraint*> m_joints;
    std::vector<b2dJsonImage*> m_images;

    std::map<cpBody*,std::string> m_bodyToNameMap;
    std::map<cpShape*,std::string> m_fixtureToNameMap;
    std::map<cpConstraint*,std::string> m_jointToNameMap;
    std::map<b2dJsonImage*,std::string> m_imageToNameMap;

    // This maps an item (cpBody*, cpShape* etc) to a set of custom properties.
    // Use NULL for world properties.
    std::map<void*,b2dJsonCustomProperties*> m_customPropertiesMap;

    // These are necessary to know what type of item the entries in the map above
    // are, which is necessary for the getBodyByCustomInt type functions.
    // We could have used a separate map for each item type, but there are many
    // combinations of item type and property type and the overall amount of
    // explicit coding to do becomes very large for no real benefit.
    std::set<cpBody*> m_bodiesWithCustomProperties;
    std::set<cpShape*> m_fixturesWithCustomProperties;
    std::set<cpConstraint*> m_jointsWithCustomProperties;
    std::set<b2dJsonImage*> m_imagesWithCustomProperties;
    std::set<cpSpace*> m_worldsWithCustomProperties;

public:
    //constructor
    b2dJson(bool useHumanReadableFloats = false);
    ~b2dJson();

    void clear();

    void setSimulationPositionIterations(int n);
    void setSimulationVelocityIterations(int n);
    void setSimulationFPS(float n);

    int getSimulationPositionIterations();
    int getSimulationVelocityIterations();
    float getSimulationFPS();

/*
    //writing functions
    Json::Value writeToValue(cpSpace* world);
    std::string writeToString(cpSpace* world);
    bool writeToFile(cpSpace* world, const char* filename);

    Json::Value b2j(cpSpace* world);
    Json::Value b2j(cpBody* body);
    Json::Value b2j(cpShape* fixture);
    Json::Value b2j(cpConstraint* joint);
    Json::Value b2j(b2dJsonImage* image);
*/
    void setBodyName(cpBody* body, const char* name);
    void setFixtureName(cpShape* fixture, const char* name);
    void setJointName(cpConstraint* joint, const char* name);
    void setImageName(b2dJsonImage* image, const char* name);

    void addImage(b2dJsonImage* image);

    //reading functions
    cpSpace* readFromValue(Json::Value worldValue);
    cpSpace* readFromString(std::string str, std::string& errorMsg);
    cpSpace* readFromFile(const char* filename, std::string& errorMsg);

    cpSpace* j2cpSpace(Json::Value worldValue);
    cpBody* j2cpBody(cpSpace* world, Json::Value bodyValue);
    cpShape* j2cpShape(cpSpace *space, cpBody *body, Json::Value fixtureValue);
    cpConstraint* j2cpConstraint(cpSpace* world, Json::Value jointValue);
    b2dJsonImage* j2b2dJsonImage(Json::Value imageValue);

    int getBodiesByName(std::string name, std::vector<cpBody*>& bodies);
    int getFixturesByName(std::string name, std::vector<cpShape*>& fixtures);
    int getJointsByName(std::string name, std::vector<cpConstraint*>& joints);
    int getImagesByName(std::string name, std::vector<b2dJsonImage*>& images);

    int getAllImages(std::vector<b2dJsonImage*>& images);

    cpBody* getBodyByName(std::string name);
    cpShape* getFixtureByName(std::string name);
    cpConstraint* getJointByName(std::string name);

    std::map<cpConstraint*,std::string> getJointToNameMap() const { return m_jointToNameMap; }
    std::map<cpShape*,std::string> getFixtureToNameMap() const { return m_fixtureToNameMap; }

    std::string getBodyName(cpBody* body);
    std::string getFixtureName(cpShape* fixture);
    std::string getJointName(cpConstraint* joint);
    std::string getImageName(b2dJsonImage* img);


    ////// custom properties

    b2dJsonCustomProperties* getCustomPropertiesForItem(void* item, bool createIfNotExisting);
protected:
    void setCustomInt(void* item, std::string propertyName, int val);
    void setCustomFloat(void* item, std::string propertyName, float val);
    void setCustomString(void* item, std::string propertyName, std::string val);
    void setCustomVector(void* item, std::string propertyName, cpVect val);
    void setCustomBool(void* item, std::string propertyName, bool val);

public:
//this define saves us writing out 25 functions which are almost exactly the same
#define DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(ucType, lcType)\
    void setCustom##ucType(cpBody* item, std::string propertyName, lcType val)          { m_bodiesWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }\
    void setCustom##ucType(cpShape* item, std::string propertyName, lcType val)       { m_fixturesWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }\
    void setCustom##ucType(cpConstraint* item, std::string propertyName, lcType val)         { m_jointsWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }\
    void setCustom##ucType(b2dJsonImage* item, std::string propertyName, lcType val)    { m_imagesWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }\
    void setCustom##ucType(cpSpace* item, std::string propertyName, lcType val)         { m_worldsWithCustomProperties.insert(item); setCustom##ucType((void*)item, propertyName, val); }

    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Int, int)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Float, float)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(String, std::string)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Vector, cpVect)
    DECLARE_SET_CUSTOM_PROPERTY_VALUE_FUNCTIONS(Bool, bool)

    bool hasCustomInt(void* item, std::string propertyName);
    bool hasCustomFloat(void* item, std::string propertyName);
    bool hasCustomString(void* item, std::string propertyName);
    bool hasCustomVector(void* item, std::string propertyName);
    bool hasCustomBool(void* item, std::string propertyName);

    int getCustomInt(void* item, std::string propertyName, int defaultVal = 0);
    float getCustomFloat(void* item, std::string propertyName, float defaultVal = 0);
    std::string getCustomString(void* item, std::string propertyName, std::string defaultVal = "");
    cpVect getCustomVector(void* item, std::string propertyName, cpVect defaultVal = cpvzero);
    bool getCustomBool(void* item, std::string propertyName, bool defaultVal = false);

//this define saves us writing out 20 functions which are almost exactly the same
#define DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(ucType, lcType)\
    int getBodiesByCustom##ucType(   std::string propertyName, lcType valueToMatch, std::vector<cpBody*>& bodies);\
    int getFixturesByCustom##ucType( std::string propertyName, lcType valueToMatch, std::vector<cpShape*>& fixtures);\
    int getJointsByCustom##ucType(   std::string propertyName, lcType valueToMatch, std::vector<cpConstraint*>& joints);\
    int getImagesByCustom##ucType(   std::string propertyName, lcType valueToMatch, std::vector<b2dJsonImage*>& images);

    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(Int, int)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(Float, float)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(String, std::string)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(Vector, cpVect)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_VECTOR(Bool, bool)

//this define saves us writing out 20 functions which are almost exactly the same
#define DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(ucType, lcType)\
    cpBody*         getBodyByCustom##ucType(    std::string propertyName, lcType valueToMatch);\
    cpShape*      getFixtureByCustom##ucType( std::string propertyName, lcType valueToMatch);\
    cpConstraint*        getJointByCustom##ucType(   std::string propertyName, lcType valueToMatch);\
    b2dJsonImage*   getImageByCustom##ucType(   std::string propertyName, lcType valueToMatch);

    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(Int, int)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(Float, float)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(String, std::string)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(Vector, cpVect)
    DECLARE_GET_BY_CUSTOM_PROPERTY_VALUE_FUNCTIONS_SINGLE(Bool, bool)

    //////


protected:
    //member helpers
    void vecToJson(const char* name, unsigned int v, Json::Value& value, int index = -1);
    void vecToJson(const char* name, float v, Json::Value& value, int index = -1);
    void vecToJson(const char* name, cpVect vec, Json::Value& value, int index = -1);
    void floatToJson(const char* name, float f, Json::Value& value);
    cpBody* lookupBodyFromIndex( unsigned int index );
    int lookupBodyIndex( cpBody* body );
    int lookupJointIndex( cpConstraint* joint );

    Json::Value writeCustomPropertiesToJson(void* item);
    void readCustomPropertiesFromJson(cpBody* item, Json::Value value);
    void readCustomPropertiesFromJson(cpShape* item, Json::Value value);
    void readCustomPropertiesFromJson(cpConstraint* item, Json::Value value);
    void readCustomPropertiesFromJson(b2dJsonImage* item, Json::Value value);
    void readCustomPropertiesFromJson(cpSpace* item, Json::Value value);

    //static helpers
    static std::string floatToHex(float f);
    static float hexToFloat(std::string str);
    static float jsonToFloat(const char* name, Json::Value& value, int index = -1, float defaultValue = 0);
    static cpVect jsonToVec(const char* name, Json::Value& value, int index = -1, cpVect defaultValue = cpvzero);
};

#endif // B2DJSON_H







