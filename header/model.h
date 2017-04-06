//
//  model.h
//  CarGame
//
//  Created by HJKBD on 8/21/16.
//  Copyright © 2016 HJKBD. All rights reserved.
//
//#include "inference.h"
#ifndef model_h
#define model_h
#include "globals.h"
#include "vec2D.h"
#include "KdTree.hpp"
#include "layout.h"
//#include "inference.h"

class Car;
using std::vector;
using std::string;
class Model;

namespace Inference {
    enum State{cooperative, aggressive};
    vector<string> intentions{"cooperative","aggressive"};
    UMAP<string, int> Intention_To_Index{{"cooperative",0},{"aggressive",1}};
    class JointParticles {
    private:
        int numParticles;
        int numAgents;
        vector<string> legalIntentions;
        vector<Car*> agents;
        Counter<vector<string>> beliefs;
        vector<vector<string>> particles;
    public:
        JointParticles(int num=600):numParticles(num),numAgents(0){};
        void initializeUniformly(const Model& model,const vector<string>& intentions);
        void initializeParticles();
        void observe(const Model& model);
        Counter<vector<string>> getBelief();
        vector<string> sample( Counter<vector<string>>& counter);
        pff getMeanStandard(queue<float>&history, const string& intention);
    };
    JointParticles jointInference = JointParticles();
    class MarginalInference {
    private:
        vector<string> legalIntentions;
        int index;
    public:
        MarginalInference(int index, const Model& model);
        void initializeUniformly(const Model& gameState);
        void observe(const Model& gameState);
        vector<float> getBelief();
    };
}

static float manhattanDistance(const Vector2f& v1, const Vector2f& v2) {
    float distance = abs(v1[0]-v2[0]) + abs(v1[1] - v2[1]);
    return distance;
}

struct Line {
    int x1,y1,x2,y2;
    Line(float _x1, float _y1, float _x2, float _y2):x1(_x1),y1(_y1),x2(_x2),y2(_y2){}
    Line(vector<int>& row) {
        x1 = row[0] * (Globals::constant.BLOCK_TILE_SIZE);
        y1 = row[1] * (Globals::constant.BLOCK_TILE_SIZE);
        x2 = row[2] * (Globals::constant.BLOCK_TILE_SIZE);
        y2 = row[3] * (Globals::constant.BLOCK_TILE_SIZE);}
    Vector2f getstart() { return Vector2f(x1, y1);}
    Vector2f getend() {return Vector2f(x2, y2);}
};

class Block {
private:
    int startx,starty;
    int endx, endy;
    float centerX, centerY;
public:
    Block(): startx(0), starty(0), endx(0),endy(0){}
    Block(vector<int>& blockdata) {
        assert(blockdata.size() == 4);
        int unit = Globals::constant.BLOCK_TILE_SIZE;
        startx = blockdata[0]*unit;
        starty = blockdata[1]*unit;
        endx = blockdata[2]*unit;
        endy = blockdata[3]*unit;
        centerX = (startx + endx)/2.0;
        centerY = (starty + endy)/2.0;
    }
    Vector2f getCenter() const { return Vector2f(centerX, centerY);}
    int getWidth() const { return abs(endx - startx); }
    int getHeight()  const{ return abs(endy - starty); }
    bool containsPoint(int x, int y) const {
        if (x < startx) return false;
        if (y < starty) return false;
        if (x > endx) return false;
        if (y > endy) return false;
        return true;
    }
    //larger one
    bool containsPointLarger(int x, int y) const {
        int size = 2;
        int startx1 = startx - size;
        int starty1 = starty - size;
        int endx1 = endx + size;
        int endy1 = endy + size;
        if (x < startx1) return false;
        if (y < starty1) return false;
        if (x > endx1) return false;
        if (y > endy1) return false;
        return true;
    }
};


// buid the model
class Model{
public:
    Model(Layout&);
    Model(const Model&);
    ~Model();
    // get the properties for the model class
    int getWidth()const { return layout.getWidth();}
    int getHeight()const { return layout.getHeight(); }
    int getBeliefRows()const{ return layout.getBeliefRows();}
    int getBeliefCols()const { return layout.getBeliefCols();}
    const vector<Block*>& getBlocks()const{ return blocks;}
    const vector<Line*>& getLine()const { return lines;}
    const vector<Car*>& getCars()const{return cars;}
    const vector<Car*>& getOtherCars()const { return otherCars; }
    vector<Vector2f> getIntersectionCenter();
    vector<Block*>& getIntersectionGraph(){ return interSections;}
    vector<Block*>& getAgentGraph(){ return agentGraph;}
    vector<Block*>& getHostGraph(){ return hostGraph; }
    vector<Block*>& getAllGraph(){  return allGraph;}
    
    Block* getIntersection(float x, float y) const;
    Block& getFinish() const {return *finish;}
    Car* getHost()const { return host; }
    void setHost(Car* car);
    
    // utility function to help check model
    bool checkVictory() const;
    bool checkCollision(Car* car) const;
    bool inBounds(float x, float y) const;
    bool inBoundsLarger(float x, float y) const;
    bool inIntersection(float x, float y) const;
    int toindex(const Car* car)const {return cartoindex.at((size_t)car);}
    
private:
    void clearBlocks(vector<Block*>&blocks);
    void initBlocks();
    void initLines();
    void initIntersections();
    void initGraphs();
    //void initOtherCars();
    //void getStartNode()
    Layout& layout;
    Block* finish;
    Car* host;
    //vector<vector<int>> othercardata;
    vector<Block*> blocks;
    vector<Line*> lines;
    vector<Car*> cars;
    vector<Car*> otherCars;
    vector<Block*> interSections;
    vector<Block*> agentGraph;
    vector<Block*> hostGraph;
    vector<Block*> allGraph;
    UMAP<size_t, int> cartoindex;
};

/*
 
 Car object initilization now
 
 */

static UMAP<string, pff> direction = {
    {"east", {1, 0}},
    {"west", {-1,0}},
    {"south",{1, 0}},
    {"north",{-1,0}},
    {"northeast", {1,1}},
    {"northwest", {-1,1}},
    {"southeast", {1,-1}},
    {"southwest", {-1,-1}}
};

//abstract class
class Car {
private:
    Vector2f pos;
    Vector2f velocity;
    Vector2f dir;
    
public:
    float wheelAngle;
    float maxSpeed;
    float friction;
    float maxWheelAngle;
    float maxaccler;
    float minSpeed;
    
public:
    void init(const string & dir);
    virtual void init();
    constexpr const static float LENGTH = 25.0;
    constexpr const  static float WIDTH = 12.5;
    const static float RADIUS;
    
    Car(){}
    
    Car(const Vector2f& _pos, string&& dir, const Vector2f& _velocity): pos(_pos),velocity(_velocity){init(dir);}
    
    Car(const Vector2f& _pos, const string& dir, const Vector2f& _velocity): pos(_pos),velocity(_velocity){init(dir);}
    
    Car(const Vector2f& _pos, const string& dir, Vector2f&& _velocity): pos(_pos),velocity(_velocity){ init(dir); }
    
    Car(const Vector2f& _pos, string&& dir, Vector2f&& _velocity): pos(_pos),velocity(_velocity) { init(dir); }
    Car(const Vector2f& _pos, const Vector2f& dir_, const Vector2f& _velocity):
    pos(_pos),dir(dir_),velocity(_velocity){init();}
    
    virtual ~Car(){};
    virtual bool isHost(){return false;}
    virtual void  autonomousAction(const vector<Vector2f>&, const Model&, kdtree::kdtree<point<float>>* tree=NULL){};
    virtual void  autonomousAction2(const vector<Vector2f>&, const Model&, int i = 1){};
    virtual void setup();
    
    Vector2f getPos() const  { return pos;}
    void setPos(const Vector2f& pos) {this->pos = pos;}
    
    Vector2f getDir() const { return dir; }
    
    Vector2f getVelocity() const { return velocity;}
    
    void turnCarTowardsWheels();
    
    void  update();
    
    void decellerate(float amount);
    
    void turnWheelsTowardsStraight(){wheelAngle = 0.0;}
    
    void applyFriction(){ decellerate(friction);}
    
    void setWheelAngle(float angle);
    
    void accelerate(float amount);
    
    vector<Vector2f> getBounds();
    
    vector<Vector2f> getBounds(Car&car, float LEN, float WID);
    
    //# http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604
    bool collides(const Vector2f& otherPos, const vector<Vector2f>& otherBounds);
    //carfufl not to too use the function, this is used for planning ahead
    void setVelocity(float amount);
    bool carInintersection(const Model& state);
    bool isCloseToOtherCar(const Model& model) const;
    
};

//derived class
class Host: public Car {
    //car model
private:
    int nodeId;
    int pre;
public:
    
    Host():Car(), nodeId{0}, pre{-1}{}
    Host(const Vector2f& _pos, string&& _dir, const Vector2f& _velocity):Car(_pos, _dir, _velocity), nodeId(0),pre(-1){setup();}
    Host(const Vector2f& _pos, const string& _dir, const Vector2f& _velocity): Car(_pos, _dir, _velocity),nodeId(0),pre(-1){setup();}
    Host(const Vector2f& _pos, const string& _dir, Vector2f&& _velocity): Car(_pos, _dir, _velocity),nodeId(0),pre(-1){setup();}
    Host(const Vector2f& _pos, string&& _dir, Vector2f&& _velocity): Car(_pos, _dir, _velocity),nodeId(0), pre(-1){setup();}
    Host(const Car& car):Car(car.getPos(), car.getDir(),car.getVelocity()),nodeId(0), pre(-1){setup();}
    ~Host(){};
    
    virtual void setup();
    bool isHost(){return true;}
    void autonomousAction(const vector<Vector2f>&path, const Model& model, kdtree::kdtree<point<float>>* tree);
    void autonomousAction2(const vector<Vector2f>&path, const Model& model, int i = 1);
    UMAP<string, float>  getAutonomousActions(const vector<Vector2f>& path, const Model& model, kdtree::kdtree<point<float>>* tree);
    UMAP<string, float>  getAutonomousActions2(const vector<Vector2f>& path, const Model& model);
    void makeObse(const Model& state);
};

//derived class for other cars
class Agent: public Car {
public:
    unsigned int timer = 0;
    bool stopflag = false;
    std::queue<float> history;
    bool hasinference;
    Inference::MarginalInference* inference;
public:
    Agent():Car(){}
    Agent(const Vector2f& _pos, string&& _dir, const Vector2f& _velocity):Car(_pos, _dir, _velocity){setup();}
    Agent(const Vector2f& _pos, const string& _dir, const Vector2f& _velocity): Car(_pos, _dir, _velocity) {setup();}
    Agent(const Vector2f& _pos, const string& _dir, Vector2f&& _velocity): Car(_pos, _dir, _velocity) {setup();}
    Agent(const Vector2f& _pos, string&& _dir, Vector2f&& _velocity): Car(_pos, _dir, _velocity){setup();}
    Agent(const Car& car):Car(car.getPos(), car.getDir(),car.getVelocity()){}
    ~Agent(){if (inference != NULL) delete inference;};
    virtual void setup();
    bool isHost(){return false;}
    std::queue<float>& getHistory(){return history;}
    void autonomousAction(const vector<Vector2f>& vec2, const Model& model, kdtree::kdtree<point<float>>* tree);
    void autonomousAction2(const vector<Vector2f>&path, const Model& model, int i = 1);
    Inference::MarginalInference* getInference(int index, const Model& model);
    Vector2f getObserv() {return getVelocity();}
};


/*
 method implementation for the models
 */

//method
Model::Model(Layout& lay):layout(lay){
    initLines();
    initBlocks();
    initGraphs();
    initIntersections();
    //initOtherCars();
    int startX = layout.getStartX();
    int startY = layout.getStartY();
    string startDir = layout.getHostDir();
    vector<int> finishdata = layout.getFinish();
    finish = new Block(finishdata);
    host =  new Host(Vector2f(startX, startY), startDir, Vector2f(0.0,0.0));
    cars.push_back(host);
    for (vector<int> other : layout.getOtherData()) {
        Car* othercar = new Agent(Vector2f(other[0],other[1]), "east", Vector2f(0.0,0.0));
        otherCars.push_back(othercar);
        cars.push_back(othercar);
    }
    cartoindex = UMAP<size_t, int>();
    for (int i = 0; i < otherCars.size(); i++)
        cartoindex.insert({(size_t)otherCars[i], i});
}

Model::Model(const Model& mo):layout(mo.layout) {
    finish = mo.finish;
    blocks = mo.blocks;
    lines = mo.lines;
    interSections = mo.interSections;
    agentGraph = mo.agentGraph;
    hostGraph = mo.hostGraph;
    allGraph = mo.allGraph;
    host = new Host(*mo.getHost());
    host->setup();
    cars.push_back(host);
    for(Car* car:mo.getOtherCars()) {
        Car* othercar = new Agent(*car);
        othercar->setup();
        otherCars.push_back(othercar);
        cars.push_back(othercar);
    }
}

Model::~Model(){
    if (cars.size() !=0) {
        while(cars.size() !=0) {
            delete cars.back();
            cars.pop_back();
        }
    }
    
}
void Model::setHost(Car* car) {
    vector<Car*> cars;
    cars.push_back(car);
    for(auto c:cars){
        if (typeid(*c) != typeid(*host))
            cars.push_back(c);
    }
    this->cars = cars;

}
void Model::clearBlocks(vector<Block*> & bloc) {
    
    if (bloc.size() != 0) {
        while( bloc.size() !=0 ) {
            delete bloc.back();
            bloc.pop_back();
        }
    }
}

void Model::initBlocks() {
    for (vector<int> blockData : layout.getBlockData()) {
        blocks.push_back(new Block(blockData));
    }
}
void Model::initLines(){
    for (vector<int> lineData :layout.getLineData())
        lines.push_back(new Line(lineData));
}

void Model::initGraphs() {
    for (vector<int> data : layout.getHostGraph()) {
        Block* hostgraph = new Block(data);
        hostGraph.push_back(hostgraph);
        allGraph.push_back(hostgraph);
    }
    for (vector<int> data : layout.getAgentGraph()) {
        Block* agentgraph = new Block(data);
        agentGraph.push_back(agentgraph);
        allGraph.push_back(agentgraph);
    }
    
}

void Model:: initIntersections(){
    for (vector<int> blockData : layout.getIntersectionData()){
        Block* inter = new Block(blockData);
        interSections.push_back(inter);
        allGraph.push_back(inter);
    }
    
}
bool Model::checkVictory() const{
    vector<Vector2f> bounds = host->getBounds();
    for(Vector2f point: bounds)
        if(finish->containsPoint(point[0], point[1]))
            return true;
    return false;
}

bool Model::checkCollision(Car* car) const{
    vector<Vector2f>bounds = car->getBounds();
    for (Vector2f point : bounds)
        if (!inBounds(point.x, point.y))
            return true;
    
    for (Car* othercar : cars) {
        if (othercar == car) continue;
        if (othercar->collides(car->getPos(), bounds))
            return true;
    }
    return false;
}
bool Model::inBounds(float x, float y) const{
    if (x < 0 || x >= getWidth()) return false;
    if (y < 0 || y >= getHeight()) return false;
    for (const auto& it : blocks)
        if (it->containsPoint(x, y))
            return false;
    return true;
}
bool Model::inBoundsLarger(float x, float y) const {
    if (x < 0 || x >= getWidth()) return false;
    if (y < 0 || y >= getHeight()) return false;
    for (const auto& it : blocks)
        if (it->containsPointLarger(x, y))
            return false;
    return true;
}
bool Model::inIntersection(float x, float y) const{
    Block* result = getIntersection(x, y);
    return result != NULL;
}

Block* Model::getIntersection(float x, float y) const{
    for (int i = 0; i < interSections.size(); i++)
        if (interSections[i]->containsPoint(x, y))
            return interSections[i];
    return NULL;
}
vector<Vector2f> Model::getIntersectionCenter() {
    vector<Vector2f>IntersectionCenter;
    for (const auto& it : interSections)
        IntersectionCenter.push_back(it->getCenter());
    return IntersectionCenter;
}



//methods and implementation
/************************************/
const float Car::RADIUS = sqrt(pow(Car::LENGTH,2) + pow(Car::WIDTH,2));
void Car::init(const string & dir) {
    pii p =direction[dir];
    this->dir = Vector2f(p.first, p.second);
    wheelAngle = 0;
    setup();
}

void Car:: init() {
    wheelAngle = 0;
    setup();
}
void Car::setup() {
    maxSpeed = 5.0;
    friction = 0.5;
    maxWheelAngle = 130.0;
    maxaccler = 2.0;
    minSpeed = 1;
}

void Car::turnCarTowardsWheels() {
    if (velocity.Length() > 0.0)
    {
    velocity.rotate(wheelAngle);
    dir = Vector2f(velocity[0], velocity[1]);
    dir.normalized();
    }
}
void Car::update() {
    turnCarTowardsWheels();
    pos += velocity;
    turnWheelsTowardsStraight();
    applyFriction();
}

void Car::decellerate(float amount) {
    float speed = velocity.Length();
    if (speed < minSpeed) {
        speed = minSpeed;
        return;
    }
    Vector2f frictionVec = velocity.get_reflection();
    frictionVec.normalized();
    frictionVec *= amount;
    velocity += frictionVec;
    float angle =  velocity.get_angle_between(frictionVec);
    if (abs(angle) < 180)
        velocity = Vector2f(0, 0);
}

void Car::setWheelAngle(float angle) {
    wheelAngle = angle;
    if (wheelAngle <= -maxWheelAngle)
        wheelAngle= -maxWheelAngle;
    if (wheelAngle >= maxWheelAngle)
        wheelAngle = maxWheelAngle;
}
void Car::accelerate(float amount) {
    amount = std::min(amount, maxaccler);
    if (amount < 0)
        decellerate(amount);
    if (amount ==0) return;
    
    Vector2f acceleration = Vector2f(dir[0], dir[1]);
    acceleration.normalized();
    acceleration *= amount;
    velocity += acceleration;
    if (velocity.Length() >= maxSpeed) {
        velocity.normalized();
        velocity *= maxSpeed;
    }
}

vector<Vector2f> Car::getBounds() {
    dir.normalized();
    Vector2f perpDir = dir.perpendicular();
    vector<Vector2f> bounds;
    bounds.push_back(pos + dir * float(LENGTH/2) + perpDir * float(WIDTH/2));
    bounds.push_back(pos + dir * float(LENGTH/2) - perpDir * float(WIDTH/2));
    bounds.push_back(pos - dir * float(LENGTH/2) + perpDir * float(WIDTH/2));
    bounds.push_back(pos - dir * float(LENGTH/2)- perpDir * float(WIDTH/2));
    return bounds;
}
vector<Vector2f> Car::getBounds(Car&car, float LEN, float WID) {
    Vector2f normalDir = normalized(car.getDir());
    Vector2f perpDir = normalDir.perpendicular();
    vector<Vector2f> bounds;
    bounds.push_back(pos + dir * float(LEN/2) + perpDir * float(WID/2));
    bounds.push_back(pos + dir * float(LEN/2) - perpDir * float(WID/2));
    bounds.push_back(pos - dir * float(LEN/2) + perpDir * float(WID/2));
    bounds.push_back(pos - dir * float(LEN/2)- perpDir * float(WID/2));
    return bounds;
}

//# http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604
bool Car::collides(const Vector2f& otherPos, const vector<Vector2f>& otherBounds) {
    Vector2f diff = otherPos - pos;
    float dist = diff.Length();
    if (dist > RADIUS * 2)
        return false;
    
    vector<Vector2f>bounds = getBounds();
    Vector2f vec1 = bounds[0] - bounds[1];
    Vector2f vec2 = otherBounds[0] - otherBounds[1];
    vector<Vector2f>  axis= {
        vec1,
        vec1.perpendicular(),
        vec2,
        vec2.perpendicular()
    };
    
    for (const auto& vec : axis) {
        pff result = projectPoints(bounds, vec);
        float minA = result.first;
        float maxA = result.second;
        result = projectPoints(otherBounds, vec);
        float minB = result.first;
        float maxB = result.second;
        bool leftmostA = (minA <= minB)?true:false;
        bool overlap = false;
        if (leftmostA && maxA >= minB) overlap = true;
        if (!leftmostA && maxB >= minA) overlap = true;
        if (!overlap) return false;
    }
    return true;
}
//carfufl not to too use the function, this is used for planning ahead
void Car::setVelocity(float amount) {
    Vector2f ve = Vector2f(dir[0], dir[1]);
    ve.normalized();
    ve *= amount;
    velocity = ve;
}
 //check car is in instersection
bool Car::carInintersection(const Model& model) {
    vector<Vector2f> bounds = getBounds();
    for (const auto& point : bounds) {
        if (model.inIntersection(point[0], point[1]))
            return true;
    }
    return false;
}


bool Car::isCloseToOtherCar(const Model& model) const {
    //### check the master car is close to others
    vector<Car*> cars = model.getCars();
    if (cars.size() == 0)
        return false;
    const Car* obstaclecar = nullptr;
    float distance = 9999999;
    for (const Car*car: cars) {
        if (car == this) continue;
        float cardis = abs(car->getPos()[0]-getPos()[0])+abs(car->getPos()[1]-getPos()[1]);
        if (cardis < distance) {
            distance = cardis;
            obstaclecar = car;
        }
    }
    
    if (!obstaclecar) return false;
    
    Vector2f diff = obstaclecar->getPos() - this->getPos();
    float angdiff = -diff.get_angle_between(this->getDir());
    if (abs(angdiff) > 90) return false;
    //std::cout<<angdiff <<std::endl;
    
    if ((abs(obstaclecar->getPos()[0] - getPos()[0]) < Globals::constant.BELIEF_TILE_SIZE*1.5) &&
        (abs(obstaclecar->getPos()[1] - getPos()[1]) < Car::WIDTH/2))
        return true;
    return false;
}


void  Host::setup() {
    maxSpeed = 3.0;
    friction = 1;
    maxWheelAngle = 45;
    maxaccler = 1.5;
    minSpeed = 1;
}
void Host:: autonomousAction(const vector<Vector2f>&path, const Model& model, kdtree::kdtree<point<float>>* tree = NULL) {
    
    if (path.size() == 0) return;
    
    Vector2f oldPos = getPos();
    Vector2f oldDir = getDir();
    //Vector2f oldVel = getVelocity();
    UMAP<string, float> actions = getAutonomousActions(path, model, tree);
    assert (getPos() == oldPos);
    assert (getDir() == oldDir);
    
//    assert (getVelocity() == oldVel);
    if (actions.count("DRIVE_FORWARD") !=0) {
        float percent = actions["DRIVE_FORWARD"];
        int sign = 1;
        if (percent < 0) sign = -1;
        percent = abs(percent);
        percent = percent > 0.0?percent:0.0;
        percent = percent < 1.0?percent:1.0;
        percent *= sign;
        accelerate(maxWheelAngle* percent);
        if (actions.count("TURN_WHEEL") != 0) {
            float turnAngle = actions["TURN_WHEEL"];
            setWheelAngle(turnAngle);
        }
    }
}

void Host:: autonomousAction2(const vector<Vector2f>&path, const Model& model, int i) {
    
    if (path.size() == 0) return;
    
    Vector2f oldPos = getPos();
    Vector2f oldDir = getDir();
    //Vector2f oldVel = getVelocity();
    UMAP<string, float> actions = getAutonomousActions2(path, model);
    assert (getPos() == oldPos);
    assert (getDir() == oldDir);
    //    assert (getVelocity() == oldVel);
    if (actions.count("DRIVE_FORWARD") !=0) {
        float percent = actions["DRIVE_FORWARD"];
        int sign = 1;
        if (percent < 0) sign = -1;
        percent = abs(percent);
        percent = percent > 0.0?percent:0.0;
        percent = percent < 1.0?percent:1.0;
        percent *= sign;
        accelerate(maxWheelAngle* percent);
        if (actions.count("TURN_WHEEL") != 0) {
            float turnAngle = actions["TURN_WHEEL"];
            setWheelAngle(turnAngle);
        }
    }
}

//bool Host::isCloseToOtherCar(const Model& model) {
//
//    vector<Car*> cars = model.getOtherCars();
//    if (cars.size() == 0) return false;
//    Car* obstaclecar = nullptr;
//    float distance = inf;
//    for (Car*car: cars) {
//        float cardis = manhattanDistance(car->getPos(), getPos());
//        if (cardis < distance) {
//            distance = cardis;
//            obstaclecar = car;
//        }
//        }
//    if (abs(obstaclecar->getPos()[0] - getPos()[0]) < Globals::constant.BELIEF_TILE_SIZE*1.5 &&
//        abs(obstaclecar->getPos()[1] - getPos()[1]) < Car::WIDTH/2)
//        return true;
//    return false;
//}

//void decisionMaking(const DecisionAgent& decision, const Model& model) {
//
//   if (isCloseToOtherCar(model))
//    action = agent.getAction(model);
//    return action;
//}
UMAP<string, float>  Host::getAutonomousActions2(const vector<Vector2f>& path, const Model& model) {
    
    UMAP<string, float> output;
    if (nodeId >= path.size())
        nodeId = 0;
    
    // set the timer to control time
    if (path.size() == 0)
        return  output;
    int nextId;
    
    Vector2f vectogoal;
    nextId = nodeId + 1;
    if (nodeId>=path.size()) nodeId = pre;
    if (nextId > path.size()) nextId = nodeId;

    Vector2f nextpos = path[nextId];
    
    if (nextpos.get_distance(getPos()) < Globals::constant.BELIEF_TILE_SIZE*0.3) {
        pre = nodeId;
        nodeId = nextId;
        nextId = nodeId + 1;
    }
    
    if (nextId >= path.size()) nextId = nodeId;
    
    //        goalPos = path[nextId];
    // we finish the checking of end point
    vectogoal = path[nextId] - getPos();
    float wheelAngle = -vectogoal.get_angle_between(getDir());
    int sign = (wheelAngle <0)?-1:1;
    wheelAngle = std::min(abs(wheelAngle), maxWheelAngle);
    
    output["TURN_WHEEL"] = wheelAngle*sign;
    output["DRIVE_FORWARD"] = 1.0;
//    if (abs(wheelAngle) < 20) output["DRIVE_FORWARD"] = 1.0;
//    else if(abs(wheelAngle) < 45) output["DRIVE_FORWARD"] = 0.8;
//    else output["DRIVE_FORWARD"] = 0.5;
    
    return output;
}

void Host::makeObse(const Model& state) {
    vector<Car*>cars = state.getOtherCars();
    for (const auto& car:cars) {
        Vector2f obsv = dynamic_cast<Agent*>(car)->getObserv();
        float obs = obsv.Length();
        obs = obs>0?obs:0;
        if (dynamic_cast<Agent*>(car)->history.size() == 11)
            dynamic_cast<Agent*>(car)->history.pop();
        dynamic_cast<Agent*>(car)->history.push(obs);
    }
}

UMAP<string, float> Host::getAutonomousActions(const vector<Vector2f>& path, const Model& model, kdtree::kdtree<point<float>>* tree) {
    
    UMAP<string, float> output;
    if (nodeId > path.size()) nodeId = 0;
    
    static unsigned int timer = 0;
    static bool stopflag = false;
    // set the timer to control time
    if (timer < 30 && stopflag) {
        setVelocity(0.0);
        output["TURN_WHEEL"] = 0;
        output["DRIVE_FORWARD"] = 0;
        timer++;
        return output;
    }
    //
    if (carInintersection(model) && !stopflag) {
        stopflag = true;
        //setVelocity(0.0);
        output["TURN_WHEEL"] = 0;
        output["DRIVE_FORWARD"] = 0;
        timer = 0;
    }
  // finished checking the
    
    if (isCloseToOtherCar(model)) {
        output["TURN_WHEEL"] = 0;
        output["DRIVE_FORWARD"] = 0;
        return output;
    }
    //
    
    if (path.size() == 0) return  output;
    int nextId;
    //= nodeId + 1;
//    if (nodeId>=path.size()) nodeId = pre;
//    if (nextId > path.size()) nextId = nodeId;
    
    Vector2f vectogoal;
    //chek the kd tree
    if ((tree != NULL) && (path.size() == tree->size())) {
        Vector2f mypos = getPos();
        vector<kdtree::node<point<float>> *> neighbors = tree->k_nearest(point<float>(mypos[0], mypos[1]), 2);
        point<float> p1 = neighbors[0]->point;
        point<float> p2 = neighbors[1]->point;
//        Vector2f v1 = Vector2f(p1.x, p1.y);
        Vector2f v2 = Vector2f(p2.x, p2.y);
//        Vector2f vectogoal1 = v1 - getPos();
        Vector2f vectogoal2 = v2 - getPos();
        float angle2 = abs(vectogoal2.get_angle_between(getDir()));
        if (angle2 < 90) nextId = p2.id;
        else nextId = p1.id;
    }else {
        nextId = nodeId + 1;
        if (nodeId>=path.size()) nodeId = pre;
        if (nextId > path.size()) nextId = nodeId;
    }
    
    Vector2f nextpos = path[nextId];
    
    if (nextpos.get_distance(getPos()) < Globals::constant.BELIEF_TILE_SIZE*0.5) {
        pre = nodeId;
        nodeId = nextId;
        nextId = nodeId + 1;
    }
    
    if (nextId >= path.size()) nextId = nodeId;
  
    // we finish the checking of end point
    vectogoal = path[nextId] - getPos();
    float wheelAngle = -vectogoal.get_angle_between(getDir());
    int sign = (wheelAngle <0)?-1:1;
    wheelAngle = std::min(abs(wheelAngle), maxWheelAngle);
    
    output["TURN_WHEEL"] = wheelAngle*sign;
    output["DRIVE_FORWARD"] = 1.0;
//    if (abs(wheelAngle) < 20) output["DRIVE_FORWARD"] = 1.0;
//    else if(abs(wheelAngle) < 45) output["DRIVE_FORWARD"] = 0.8;
//    else output["DRIVE_FORWARD"] = 0.5;
    
    return output;
}


void Agent::setup()
{
    maxSpeed = 3.0;
    friction = 1;
    maxWheelAngle = 45;
    maxaccler = 1.4;
    minSpeed = 1;
    history = std::queue<float>();
    hasinference = false;
    inference = NULL;
}


void Agent::autonomousAction(const vector<Vector2f>& vec2, const Model& model, kdtree::kdtree<point<float>>* tree){
    /*
     here we have three choices to choose: normal, acc, dec
     */
    // set the timer to control time
    if (timer < 30 && stopflag) {
        //setVelocity(0.0);
        timer++;
        return;
    }
    //
    bool check = carInintersection(model);
    if (check && !stopflag) {
        stopflag = true;
        accelerate(0);
        setWheelAngle(0);
        timer = 0;
        return;
    }
    
    if (isCloseToOtherCar(model)) {
        accelerate(0);
        setWheelAngle(0);
        return;
    }
    
    //unsigned int i = rand()%1;
    //assume it is not conservative for all drivers
    unsigned int i = 1;
    Car* host = model.getHost();
    //conservative driver will yield
    if ((host->getPos().x < this->getPos().x + Car::LENGTH*4) && (host->getPos().x > this->getPos().x))
        i = 0;
    switch (i) {
        case 0:
            accelerate(friction);
            setWheelAngle(0);
            break;
        case 1:
            accelerate(maxaccler);
            setWheelAngle(0);
            break;
        case 2:
            accelerate(maxaccler*0.25);
            setWheelAngle(0);
            break;
        default:
            break;
    }
}
void Agent::autonomousAction2(const vector<Vector2f>& vec2, const Model& model, int i){
    
    //unsigned int i = rand()%1;
    //assume it is not conservative for all drivers
    switch (i) {
        case 0:
            accelerate(friction);
            setWheelAngle(0);
            break;
        case 1:
            accelerate(maxaccler);
            setWheelAngle(0);
            break;
        default:
            break;
    }
}
Inference::MarginalInference* Agent::getInference(int index, const Model& state) {
    if (!hasinference) {
        inference = new Inference::MarginalInference(index, state);
        hasinference = true;
        return inference;
    }
    return inference;
}
#endif /* model_h */
