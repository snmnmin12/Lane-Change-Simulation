#ifndef INFERENCE_H
#define INFERENCE_H
#include "model.h"
#include "globals.h"
namespace Inference {

    //a helper function to produce the permuation of a list of states
    vector<vector<string>> product(const vector<string>& states, int repeat = 2) {
        vector<vector<string>> res(states.size());
        vector<vector<string>> output;
        for (int i = 0; i < states.size(); i++)
            res[i].push_back(states[i]);
        
        if (repeat == 1) { return res;}
        vector<vector<string>> middle = product(states, repeat - 1);
        for (int i = 0; i < res.size(); i++) {
            for (int j = 0; j < middle.size(); j++) {
                vector<string> temp(res[i]);
                temp.insert(temp.end(), middle[j].begin(),middle[j].end());
                output.push_back(temp);
            }
        }
        return output;
    }
    
    //to produce pdf
    double pdf(float mean, float std, float value) {
        double u = double(value - mean)/abs(std);
        double y = (1.0 / (sqrt(2 * PI) * abs(std))) * exp(-u * u / 2.0);
        if (y == 0)
            return 0.00000001;
//            cout<<"I am here!"<<endl;
        return y;
    }
    
    
    
 //started writing the classes for the simulations above
    class Belief
    {
    private:
        vector<float> grid;
        int numElems;
    public:
        Belief(int _numElems, float value = -1):numElems(_numElems){
            if (value == -1) value = (1.0 / numElems);
            grid.resize(numElems);
            std::fill(grid.begin(), grid.end(),value);
        }
        float operator[](int i) { return grid[i];}
        void setProb(int row, float p) { grid[row] = p; }
        void addProb(int row, float delta) {
            grid[row] += delta;
            assert(grid[row] >= 0.0);
        }
    // Returns the belief for tile row, col.
        float getProb(int row) {return grid[row];}
    
        // Function: Normalize
        void normalize() {
            float  total = getSum();
            for (int i = 0; i< numElems; i++)
            grid[i] /= total;
        }
        int getNumElems() { return numElems;}
    
        float getSum() {
            float total = 0.0;
            for (int i = 0; i < numElems; i++)
            total += getProb(i);
            return total;
        }
    };

    
    void JointParticles::initializeUniformly(const Model& model, const vector<string>& intentions) {
        //stores infomraiton about the game, then initialize the particles
        numAgents = model.getOtherCars().size();
        legalIntentions = intentions;
        beliefs = Counter<vector<string>>();
        initializeParticles();
    }
    
    void JointParticles::initializeParticles() {
        std::random_device rd;
        std::mt19937 g(rd());
        vector<vector<string>> jointstates = product(legalIntentions, numAgents);
        std::shuffle (jointstates.begin(), jointstates.end(), g);
        int n = numParticles;
        int p = jointstates.size();
        particles.clear();
        while (n > p) {
            particles.insert(particles.end(), jointstates.begin(), jointstates.end());
            n -= p;
        }
        particles.insert(particles.end(),jointstates.begin(),jointstates.begin()+n);
    }
    
    void JointParticles::observe(const Model& model) {
        if (beliefs.size() == 1)
            initializeParticles();
        vector<Car*> othercars = model.getOtherCars();
        Counter<vector<string>> tempCounter = Counter<vector<string>>();
        for (int i = 0; i < particles.size(); i++) {
            float prob = 1;
            vector<string> intentions = particles[i];
            for (int index = 0; index < numAgents; index++) {
                queue<float> history = ((Agent*)othercars[index])->getHistory();
                float observ = history.back();
                string intention = intentions[index];
                pff res = getMeanStandard(history, intention);
                prob *= pdf(res.first, res.second, observ);
            }
            tempCounter[intentions] += prob;
        }
        
        beliefs = tempCounter;
        for(const auto& item:beliefs) {
            for (int i = 0; i < item.first.size(); i++)
                cout<<item.first[i]<<" ";
                cout<<item.second<<endl;
        }
        cout<<"Now it has finished!"<<endl;
        //resampling
        if (tempCounter.size() == 0)
            initializeParticles();
        else {
            beliefs.normalize();
            for (int i = 0; i < particles.size(); i++) {
                vector<string> newPos = sample(beliefs);
                particles[i] = newPos;
            }
        }
    }
    
    Counter<vector<string>> JointParticles::getBelief() {
        Counter<vector<string>> beliefDist = Counter<vector<string>>();
        for (int index = 0; index < particles.size(); index++)
            beliefDist[particles[index]] +=1;
        beliefDist.normalize();
        return beliefDist;
    }

   vector<string> JointParticles::sample(Counter<vector<string>>& distribution) {
        if (distribution.sum() != 1) distribution.normalize();
        std::vector<std::pair<vector<string>, float>> elems(distribution.begin(), distribution.end());
        std::sort(elems.begin(), elems.end(),
                  [](const std::pair<vector<string>, float>& a, const std::pair<vector<string>, float>& b)->bool{return a.second<b.second;});
        vector<vector<string>> keys;
        vector<float> values;
        for (auto item : elems) {
            keys.push_back(item.first);
            values.push_back(item.second);
        }
        double choice = ((double) rand() / (RAND_MAX));
        int i = 0;
        double total = values[0];
        while (choice > total) {
            i += 1;
            total += values[i];
        }
       return keys[i>keys.size()-1?keys.size()-1:i];
    }
    pff JointParticles::getMeanStandard(queue<float>&history, const string& intention) {
        int total = history.front();
        float vref = total;
        if (vref == 0) vref = 0.01;
        float sigma = 0.3*vref;
        int index = Intention_To_Index[intention];
        if  (index == 0)
            return pff(0.7*vref, sigma);
        else if (index == 1)
            return pff(vref, sigma);
        return pff(0, 0);
    }

    MarginalInference::MarginalInference(int index, const Model& model) {
        this->index = index;
        legalIntentions = intentions;
        initializeUniformly(model);
    }
    
    void MarginalInference::initializeUniformly(const Model& gameState) {
        if (index == 1)
            jointInference.initializeUniformly(gameState, legalIntentions);
    }
    
    void MarginalInference::observe(const Model& gameState) {
        if (index == 1)
            jointInference.observe(gameState);
    }
    std::vector<float> MarginalInference::getBelief() {
        Counter<vector<string>> jointDistribution = jointInference.getBelief();
        Counter<int> dist = Counter<int>();
        for (const auto& item: jointDistribution) {
            int i = Intention_To_Index[item.first[index-1]];
            dist[i] += item.second;
        }
        vector<float> result = vector<float>();
        result.resize(legalIntentions.size());
        for (const auto& item: dist)
            result[item.first] = item.second;
        return result;
    }
}
#endif
