
Object.prototype.hasOwnProperty = function(property) {
    return typeof(this[property]) !== 'undefined'
};

function loadBodyFromRUBE(bodyJso, world) {
    //console.log(bodyJso);
    
    if ( ! bodyJso.hasOwnProperty('type') ) {
        console.log("Body does not have a 'type' property");
        return null;
    }    
    
    var bd = new b2BodyDef();
    if ( bodyJso.type == 2 )
        bd.type = b2_dynamicBody;
    else if ( bodyJso.type == 1 )
        bd.type = b2_kinematicBody;
    if ( bodyJso.hasOwnProperty('angle') )
        bd.angle = bodyJso.angle;
    if ( bodyJso.hasOwnProperty('angularVelocity') )
        bd.angularVelocity = bodyJso.angularVelocity;
    if ( bodyJso.hasOwnProperty('active') )
        bd.awake = bodyJso.active;        
    if ( bodyJso.hasOwnProperty('fixedRotation') )
        bd.fixedRotation = bodyJso.fixedRotation;
    if ( bodyJso.hasOwnProperty('linearVelocity') && bodyJso.linearVelocity instanceof Object )
        bd.linearVelocity.SetV( bodyJso.linearVelocity );
    if ( bodyJso.hasOwnProperty('position') && bodyJso.position instanceof Object )
        bd.position.SetV( bodyJso.position );
    if ( bodyJso.hasOwnProperty('awake') )
        bd.awake = bodyJso.awake;
    else
        bd.awake = false;
    var body = world.CreateBody(bd);
    if ( bodyJso.hasOwnProperty('fixture') ) {
        for (k = 0; k < bodyJso['fixture'].length; k++) {
            var fixtureJso = bodyJso['fixture'][k];
            loadFixtureFromRUBE(body, fixtureJso);
        }
    }
    if ( bodyJso.hasOwnProperty('name') )
        body.name = bodyJso.name;
    if ( bodyJso.hasOwnProperty('customProperties') )
        body.customProperties = bodyJso.customProperties;
    return body;
}

function loadFixtureFromRUBE(body, fixtureJso) {    
    //console.log(fixtureJso);
    var fd = new b2FixtureDef();
    if (fixtureJso.hasOwnProperty('friction'))
        fd.friction = fixtureJso.friction;
    if (fixtureJso.hasOwnProperty('density'))
        fd.density = fixtureJso.density;
    if (fixtureJso.hasOwnProperty('restitution'))
        fd.restitution = fixtureJso.restitution;
    if (fixtureJso.hasOwnProperty('sensor'))
        fd.isSensor = fixtureJso.sensor;
    if ( fixtureJso.hasOwnProperty('filter-categoryBits') )
        fd.filter.categoryBits = fixtureJso['filter-categoryBits'];
    if ( fixtureJso.hasOwnProperty('filter-maskBits') )
        fd.filter.maskBits = fixtureJso['filter-maskBits'];
    if ( fixtureJso.hasOwnProperty('filter-groupIndex') )
        fd.filter.groupIndex = fixtureJso['filter-groupIndex'];
    if (fixtureJso.hasOwnProperty('circle')) {
        fd.shape = new b2CircleShape();
        fd.shape.m_radius = fixtureJso.circle.radius;
        if ( fixtureJso.circle.center )
            fd.shape.m_p.SetV(fixtureJso.circle.center);
        var fixture = body.CreateFixture(fd);        
        if ( fixtureJso.name )
            fixture.name = fixtureJso.name;
    }
    else if (fixtureJso.hasOwnProperty('polygon')) {
        fd.shape = new b2PolygonShape();
        var verts = [];
        for (v = 0; v < fixtureJso.polygon.vertices.x.length; v++) 
           verts.push( new b2Vec2( fixtureJso.polygon.vertices.x[v], fixtureJso.polygon.vertices.y[v] ) );
        fd.shape.SetAsArray(verts, verts.length);
        var fixture = body.CreateFixture(fd);        
        if ( fixture && fixtureJso.name )
            fixture.name = fixtureJso.name;
    }
    else if (fixtureJso.hasOwnProperty('chain')) {
        fd.shape = new b2PolygonShape();
        var lastVertex = new b2Vec2();
        for (v = 0; v < fixtureJso.chain.vertices.x.length; v++) {
            var thisVertex = new b2Vec2( fixtureJso.chain.vertices.x[v], fixtureJso.chain.vertices.y[v] );
            if ( v > 0 ) {
                fd.shape.SetAsEdge( lastVertex, thisVertex );
                var fixture = body.CreateFixture(fd);        
                if ( fixtureJso.name )
                    fixture.name = fixtureJso.name;
            }
            lastVertex = thisVertex;
        }
    }
    else {
        console.log("Could not find shape type for fixture");
    }
}

function getVectorValue(val) {
    if ( val instanceof Object )
        return val;
    else
        return { x:0, y:0 };
}

function loadJointCommonProperties(jd, jointJso, loadedBodies) {    
    jd.bodyA = loadedBodies[jointJso.bodyA];
    jd.bodyB = loadedBodies[jointJso.bodyB];
    jd.localAnchorA.SetV( getVectorValue(jointJso.anchorA) );
    jd.localAnchorB.SetV( getVectorValue(jointJso.anchorB) );
    if ( jointJso.collideConnected )
        jd.collideConnected = jointJso.collideConnected;
}

function loadJointFromRUBE(jointJso, world, loadedBodies)
{
    if ( ! jointJso.hasOwnProperty('type') ) {
        console.log("Joint does not have a 'type' property");
        return null;
    }    
    if ( jointJso.bodyA >= loadedBodies.length ) {
        console.log("Index for bodyA is invalid: " + jointJso.bodyA );
        return null;
    }    
    if ( jointJso.bodyB >= loadedBodies.length ) {
        console.log("Index for bodyB is invalid: " + jointJso.bodyB );
        return null;
    }
    
    var joint = null;
    if ( jointJso.type == "revolute" ) {
        var jd = new b2RevoluteJointDef();
        loadJointCommonProperties(jd, jointJso, loadedBodies);
        if ( jointJso.hasOwnProperty('refAngle') )
            jd.referenceAngle = jointJso.refAngle;
        if ( jointJso.hasOwnProperty('lowerLimit') )
            jd.lowerAngle = jointJso.lowerLimit;
        if ( jointJso.hasOwnProperty('upperLimit') )
            jd.upperAngle = jointJso.upperLimit;
        if ( jointJso.hasOwnProperty('maxMotorTorque') )
            jd.maxMotorTorque = jointJso.maxMotorTorque;
        if ( jointJso.hasOwnProperty('motorSpeed') )
            jd.motorSpeed = jointJso.motorSpeed;
        if ( jointJso.hasOwnProperty('enableLimit') )
            jd.enableLimit = jointJso.enableLimit;
        if ( jointJso.hasOwnProperty('enableMotor') )
            jd.enableMotor = jointJso.enableMotor;
        joint = world.CreateJoint(jd);
    }
    else if ( jointJso.type == "distance" ) {
        var jd = new b2DistanceJointDef();
        loadJointCommonProperties(jd, jointJso, loadedBodies);
        if ( jointJso.hasOwnProperty('length') )
            jd.length = jointJso.length;
        if ( jointJso.hasOwnProperty('dampingRatio') )
            jd.dampingRatio = jointJso.dampingRatio;
        if ( jointJso.hasOwnProperty('frequency') )
            jd.frequencyHz = jointJso.frequency;
        joint = world.CreateJoint(jd);
    }
    else if ( jointJso.type == "prismatic" ) {
        var jd = new b2PrismaticJointDef();
        loadJointCommonProperties(jd, jointJso, loadedBodies);        
        if ( jointJso.hasOwnProperty('localAxisA') )
            jd.localAxisA.SetV( getVectorValue(jointJso.localAxisA) );         
        if ( jointJso.hasOwnProperty('refAngle') )
            jd.referenceAngle = jointJso.refAngle;
        if ( jointJso.hasOwnProperty('enableLimit') )
            jd.enableLimit = jointJso.enableLimit;
        if ( jointJso.hasOwnProperty('lowerLimit') )
            jd.lowerTranslation = jointJso.lowerLimit;
        if ( jointJso.hasOwnProperty('upperLimit') )
            jd.upperTranslation = jointJso.upperLimit;
        if ( jointJso.hasOwnProperty('enableMotor') )
            jd.enableMotor = jointJso.enableMotor;
        if ( jointJso.hasOwnProperty('maxMotorForce') )
            jd.maxMotorForce = jointJso.maxMotorForce;
        if ( jointJso.hasOwnProperty('motorSpeed') )
            jd.motorSpeed = jointJso.motorSpeed;            
        joint = world.CreateJoint(jd);
    }
    else if ( jointJso.type == "wheel" ) {
        //Make a fake wheel joint using a line joint and a distance joint.
        //Return the line joint because it has the linear motor controls.
        //Use ApplyTorque on the bodies to spin the wheel...
        
        var jd = new b2DistanceJointDef();
        loadJointCommonProperties(jd, jointJso, loadedBodies);
        jd.length = 0.0;
        if ( jointJso.hasOwnProperty('springDampingRatio') )
            jd.dampingRatio = jointJso.springDampingRatio;
        if ( jointJso.hasOwnProperty('springFrequency') )
            jd.frequencyHz = jointJso.springFrequency;
        world.CreateJoint(jd);
        
        jd = new b2LineJointDef();
        loadJointCommonProperties(jd, jointJso, loadedBodies);
        if ( jointJso.hasOwnProperty('localAxisA') )
            jd.localAxisA.SetV( getVectorValue(jointJso.localAxisA) );
            
        joint = world.CreateJoint(jd);
    }
    else {
        console.log("Unsupported joint type: " + jointJso.type);
        console.log(jointJso);
    }
    if ( joint && jointJso.name )
        joint.name = jointJso.name;
    return joint;
}

function makeClone(obj) {
  var newObj = (obj instanceof Array) ? [] : {};
  for (var i in obj) {
    if (obj[i] && typeof obj[i] == "object") 
      newObj[i] = makeClone(obj[i]);
    else
        newObj[i] = obj[i];
  }
  return newObj;
};

function loadImageFromRUBE(imageJso, world, loadedBodies)
{
    var image = makeClone(imageJso);
    
    if ( image.hasOwnProperty('body') && image.body >= 0 )
        image.body = loadedBodies[image.body];//change index to the actual body
    else
        image.body = null;
        
    image.center = new b2Vec2();
    image.center.SetV( getVectorValue(imageJso.center) );
    
    return image;
}



//mainly just a convenience for the testbed - uses global 'world' variable
function loadSceneFromRUBE(worldJso) {
    return loadSceneIntoWorld(worldJso, world);
}

//load the scene into an already existing world variable
function loadSceneIntoWorld(worldJso, world) {
    var success = true;
    
    var loadedBodies = [];
    if ( worldJso.hasOwnProperty('body') ) {
        for (var i = 0; i < worldJso.body.length; i++) {
            var bodyJso = worldJso.body[i];
            var body = loadBodyFromRUBE(bodyJso, world);
            if ( body )
                loadedBodies.push( body );
            else
                success = false;
        }
    }
    
    var loadedJoints = [];
    if ( worldJso.hasOwnProperty('joint') ) {
        for (var i = 0; i < worldJso.joint.length; i++) {
            var jointJso = worldJso.joint[i];
            var joint = loadJointFromRUBE(jointJso, world, loadedBodies);
            if ( joint )
                loadedJoints.push( joint );
            else
                success = false;
        }
    }
    
    var loadedImages = [];
    if ( worldJso.hasOwnProperty('image') ) {
        for (var i = 0; i < worldJso.image.length; i++) {
            var imageJso = worldJso.image[i];
            var image = loadImageFromRUBE(imageJso, world, loadedBodies);
            if ( image )
                loadedImages.push( image );
            else
                success = false;
        }        
        world.images = loadedImages;
    }
    
    return success;
}

//create a world variable and return it if loading succeeds
function loadWorldFromRUBE(worldJso) {
    var gravity = new b2Vec2(0,0);
    if ( worldJso.hasOwnProperty('gravity') && worldJso.gravity instanceof Object )
        gravity.SetV( worldJso.gravity );
    var world = new b2World( gravity );
    if ( ! loadSceneIntoWorld(worldJso, world) )
        return false;
    return world;
}

function getNamedBodies(world, name) {
    var bodies = [];
    for (b = world.m_bodyList; b; b = b.m_next) {
        if ( b.name == name )
            bodies.push(b);
    }
    return bodies;
}

function getNamedFixtures(world, name) {
    var fixtures = [];
    for (b = world.m_bodyList; b; b = b.m_next) {
        for (f = b.m_fixtureList; f; f = f.m_next) {
            if ( f.name == name )
                fixtures.push(f);
        }
    }
    return fixtures;
}

function getNamedJoints(world, name) {
    var joints = [];
    for (j = world.m_jointList; j; j = j.m_next) {
        if ( j.name == name )
            joints.push(j);
    }
    return joints;
}

function getNamedImages(world, name) {
    var images = [];
    for (i = 0; i < world.images.length; i++) {
        if ( world.images[i].name == name )
            images.push(world.images[i].name);
    }
    return images;
}

//custom properties
function getBodiesByCustomProperty(world, propertyType, propertyName, valueToMatch) {
    var bodies = [];
    for (b = world.m_bodyList; b; b = b.m_next) {
        if ( ! b.hasOwnProperty('customProperties') )
            continue;
        for (var i = 0; i < b.customProperties.length; i++) {
            if ( ! b.customProperties[i].hasOwnProperty("name") )
                continue;
            if ( ! b.customProperties[i].hasOwnProperty(propertyType) )
                continue;
            if ( b.customProperties[i].name == propertyName &&
                 b.customProperties[i][propertyType] == valueToMatch)
                bodies.push(b);
        }        
    }
    return bodies;
}

function hasCustomProperty(item, propertyType, propertyName) {
    if ( !item.hasOwnProperty('customProperties') )
        return false;
    for (var i = 0; i < item.customProperties.length; i++) {
        if ( ! item.customProperties[i].hasOwnProperty("name") )
            continue;
        if ( ! item.customProperties[i].hasOwnProperty(propertyType) )
            continue;
        return true;
    }
    return false;
}

function getCustomProperty(item, propertyType, propertyName, defaultValue) {
    if ( !item.hasOwnProperty('customProperties') )
        return defaultValue;
    for (var i = 0; i < item.customProperties.length; i++) {
        if ( ! item.customProperties[i].hasOwnProperty("name") )
            continue;
        if ( ! item.customProperties[i].hasOwnProperty(propertyType) )
            continue;
        if ( item.customProperties[i].name == propertyName )
            return item.customProperties[i][propertyType];
    }
    return defaultValue;
}











