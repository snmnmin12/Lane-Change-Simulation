//
//  model2.h
//  CarGame
//
//  Created by HJKBD on 11/19/16.
//  Copyright © 2016 HJKBD. All rights reserved.
//

#ifndef model_h
#define model_h

#include "globals.h"
#include "vec2D.h"
#include "KdTree.hpp"
#include "layout.h"
#include "inference.h"

class MarginalInference;
class Car;
class Model;
using std::vector;
using std::string;

static float manhattanDistance(const Vector2f& v1, const Vector2f& v2) {
    float distance = abs(v1[0]-v2[0]) + abs(v1[1] - v2[1]);
    return distance;
}

struct Line {
    int x1,y1,x2,y2;
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
    virtual void  autonomousAction2(const vector<Vector2f>&, const Model&, kdtree::kdtree<point<float>>* tree=NULL){};
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
    void autonomousAction2(const vector<Vector2f>&path, const Model& model, kdtree::kdtree<point<float>>* tree);
    UMAP<string, float>  getAutonomousActions(const vector<Vector2f>& path, const Model& model, kdtree::kdtree<point<float>>* tree);
    UMAP<string, float>  getAutonomousActions2(const vector<Vector2f>& path, const Model& model, kdtree::kdtree<point<float>>* tree);
    
};

//derived class for other cars
class Agent: public Car {
public:
    unsigned int timer = 0;
    bool stopflag = false;
    std::queue<float> history;
    bool hasinference;
    MarginalInference* inference;
public:
    Agent():Car(){}
    Agent(const Vector2f& _pos, string&& _dir, const Vector2f& _velocity):Car(_pos, _dir, _velocity){setup();}
    Agent(const Vector2f& _pos, const string& _dir, const Vector2f& _velocity): Car(_pos, _dir, _velocity) {setup();}
    Agent(const Vector2f& _pos, const string& _dir, Vector2f&& _velocity): Car(_pos, _dir, _velocity) {setup();}
    Agent(const Vector2f& _pos, string&& _dir, Vector2f&& _velocity): Car(_pos, _dir, _velocity){setup();}
    Agent(const Car& car):Car(car.getPos(), car.getDir(),car.getVelocity()){}
    ~Agent(){};
    virtual void setup();
    bool isHost(){return false;}
    std::queue<float>& getHistory(){return history;}
    void autonomousAction(const vector<Vector2f>& vec2, const Model& model, kdtree::kdtree<point<float>>* tree);
    MarginalInference* getInference(int index, const Model& model);
};

//namespace Inference {
//    enum State{cooperative, normal, aggressive};
//    vector<string> intentions{"cooperative","normal","aggressive"};
//    UMAP<string, int> Intention_To_Index{{"cooperative",0},{"normal",1},{"aggressive",2}};
//    
//    //to produce the permuation of a list of states
//    vector<vector<string>> product(const vector<string>& states, int repeat = 3) {
//        vector<vector<string>> res(states.size());
//        vector<vector<string>> output;
//        for (int i = 0; i < states.size(); i++)
//            res[i].push_back(states[i]);
//        
//        if (repeat == 1) { return res;}
//        vector<vector<string>> middle = product(states, repeat - 1);
//        for (int i = 0; i < res.size(); i++) {
//            for (int j = 0; j < middle.size(); j++) {
//                vector<string> temp(res[i]);
//                temp.insert(temp.end(), middle[j].begin(),middle[j].end());
//                output.push_back(temp);
//            }
//        }
//        return output;
//    }
//    
//    //to produce pdf
//    double pdf(float mean, float std, float value) {
//        double u = double(value - mean)/abs(std);
//        double y = (1.0 / (sqrt(2 * M_PI) * abs(std))) * exp(-u * u / 2.0);
//        return y;
//    }
//    
//    
//    
//    //started writing the classes for the simulations above
//    class Belief
//    {
//    private:
//        vector<float> grid;
//        int numElems;
//    public:
//        Belief(int _numElems, float value = -1):numElems(_numElems){
//            if (value == -1) value = (1.0 / numElems);
//            grid.resize(numElems);
//            std::fill(grid.begin(), grid.end(),value);
//        }
//        float operator[](int i) { return grid[i];}
//        void setProb(int row, float p) { grid[row] = p; }
//        void addProb(int row, float delta) {
//            grid[row] += delta;
//            assert(grid[row] >= 0.0);
//        }
//        // Returns the belief for tile row, col.
//        float getProb(int row) {return grid[row];}
//        
//        // Function: Normalize
//        void normalize() {
//            float  total = getSum();
//            for (int i = 0; i< numElems; i++)
//                grid[i] /= total;
//        }
//        int getNumElems() { return numElems;}
//        
//        float getSum() {
//            float total = 0.0;
//            for (int i = 0; i < numElems; i++)
//                total += getProb(i);
//            return total;
//        }
//    };
//    
//    //to build the joint particles for processing
//    class JointParticles {
//    private:
//        int numParticles;
//        int numAgents;
//        vector<string> legalIntentions;
//        vector<Car*> agents;
//        Counter<vector<string>> beliefs;
//        vector<vector<string>> particles;
//    public:
//        JointParticles(int num=600):numParticles(num),numAgents(0){};
//        void initializeUniformly(const Model& model,const vector<string>& intentions);
//        void initializeParticles();
//        void observe(const Model& model);
//        Counter<vector<string>> getBelief();
//        vector<string> sample( Counter<vector<string>>& counter);
//        pff getMeanStandard(queue<float>&history, const string& intention);
//    };
//    
//    void JointParticles::initializeUniformly(const Model& model, const vector<string>& intentions) {
//        //stores infomraiton about the game, then initialize the particles
//        numAgents = model.getOtherCars().size();
//        legalIntentions = intentions;
//        beliefs = Counter<vector<string>>();
//        initializeParticles();
//    }
//    
//    void JointParticles::initializeParticles() {
//        std::random_device rd;
//        std::mt19937 g(rd());
//        vector<vector<string>> jointstates = product(legalIntentions, numAgents);
//        std::shuffle (jointstates.begin(), jointstates.end(), g);
//        int n = numParticles;
//        int p = jointstates.size();
//        while (n > p) {
//            particles.insert(particles.end(), jointstates.begin(), jointstates.end());
//            n -= p;
//        }
//        particles.insert(particles.end(),jointstates.begin(),jointstates.begin()+n-1);
//    }
//    void JointParticles::observe(const Model& model) {
//        if (beliefs.size() == 1) initializeParticles();
//        vector<Car*> othercars = model.getOtherCars();
//        Counter<vector<string>> tempCounter = Counter<vector<string>>();
//        for (int i = 0; i < particles.size(); i++) {
//            float prob = 1;
//            vector<string> intentions = particles[i];
//            for (int index = 0; index < numAgents; index++) {
//                queue<float> history = ((Agent*)othercars[i])->getHistory();
//                float observ = history.front();
//                string intention = intentions[index];
//                pff res = getMeanStandard(history, intention);
//                prob *= pdf(res.first, res.second, observ);
//            }
//        }
//        
//        beliefs = tempCounter;
//        
//        //resampling
//        if (tempCounter.size() == 0)
//            initializeParticles();
//        else
//            beliefs.normalize();
//        // print self.beliefs
//        for (int i = 0; i < particles.size(); i++) {
//            vector<string> newPos = sample(beliefs);
//            particles[i] = newPos;
//        }
//    }
//    
//    Counter<vector<string>> JointParticles::getBelief() {
//        Counter<vector<string>> beliefDist = Counter<vector<string>>();
//        for (int index = 0; index < particles.size(); index++) {
//            beliefDist[particles[index]] +=1;
//            beliefDist.normalize();
//        }
//        return beliefDist;
//    }
//    
//    vector<string> JointParticles::sample(Counter<vector<string>>& distribution) {
//        if (distribution.sum() != 1) distribution.normalize();
//        std::vector<std::pair<vector<string>, float>> elems(distribution.begin(), distribution.end());
//        std::sort(elems.begin(), elems.end(),
//                  [](const std::pair<vector<string>, float>& a, const std::pair<vector<string>, float>& b)->bool{return a.second<b.second;});
//        vector<vector<string>> keys;
//        vector<float> values;
//        for (auto item : elems) {
//            keys.push_back(item.first);
//            values.push_back(item.second);
//        }
//        double choice = ((double) rand() / (RAND_MAX));
//        int i = 0;
//        double total = values[0];
//        while (choice > total) {
//            i += 1;
//            total += values[i];
//        }
//        return keys[i];
//    }
//    pff  JointParticles::getMeanStandard(queue<float>&history, const string& intention) {
//        int total = history.back();
//        //        for (int i = 0; i < history.size(); i++)
//        //            total += history[i];
//        
//        float vref =  total/history.size();
//        if (vref == 0) vref = 0.001;
//        float sigma = 0.5*vref;
//        int index = Intention_To_Index[intention];
//        if  (index == 0)
//            return pff(0, sigma);
//        else if (index == 1)
//            return pff(0.5*vref, sigma);
//        else if (index == 2)
//            return pff(vref,sigma);
//        else if (index == 3)
//            return pff(1.5*vref, sigma);
//        return pff(0, 0);
//    }
//    //to construct a joint inference array
//    JointParticles jointInference = JointParticles();
//    class MarginalInference {
//    private:
//        vector<string> legalIntentions;
//        int index;
//    public:
//        MarginalInference(int index, const Model& model) {
//            this->index = index;
//            legalIntentions = intentions;
//            initializeUniformly(model);
//        }
//        
//        void initializeUniformly(const Model& gameState) {
//            if (index == 1)
//                jointInference.initializeUniformly(gameState, legalIntentions);
//        }
//        
//        void observe(const Model& gameState) {
//            if (index == 1)
//                jointInference.observe(gameState);
//        }
//        vector<float> getBelief() {
//            Counter<vector<string>> jointDistribution = jointInference.getBelief();
//            Counter<int> dist = Counter<int>();
//            for (const auto& item: jointDistribution) {
//                int i = Intention_To_Index[item.first[index-1]];
//                dist[i] += item.second;
//            }
//            vector<float> result = vector<float>();
//            result.resize(legalIntentions.size());
//            for (const auto& item: dist)
//                result[item.first] = item.second;
//            return result;
//        }
//    };
//}
#endif /* model2_h */
