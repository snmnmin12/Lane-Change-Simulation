#ifndef GLOBALS_H
#define GLOBALS_H
        
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <random>
#include "vec2D.h"
using namespace std;

#define inf std::numeric_limits<double>::infinity()
#define PI 3.14159265
#define MAP map
#define UMAP unordered_map
typedef std::pair<int, int> pii;
typedef std::pair<float, float> pff;
typedef Vector2d<float> Vector2f;



//set counter class
template<class Key>
class Counter:public std::UMAP<Key, float>
{
public:
    float & operator[]( const Key& key ) {
        if(this->count(key) == 0 )
            this->insert({key, 0.0});
        return this->at(key);
    }
    float& operator[]( Key&& key ) {
        if(this->count(key) == 0)
            this->insert({key, 0.0});
        return this->at(key);
    }
    vector<float> values() {
        vector<float> val;
        val.reserve(this->size());
        for (auto it = this->begin(); it != this->end; it++) {
            val.push_back(it->second);
        }
        return val;
    }
    float sum() {
        float total = 0;
        for (auto it = this->begin(); it != this->end(); it++)
            total += it->second;
        return total;
    }
    void normalize() {
        float total = sum();
        for (auto it = this->begin(); it != this->end(); it++)
            this->at(it->first) = it->second/total;
//             it->second = it->second/total;
    }
};


namespace Globals {
    extern const double INF;
    extern const double TINY;
    struct Constant {
        string TITLE;
        float SONAR_STD;
        string LAYOUT_DIR;
        int BLOCK_TILE_SIZE;
        int BELIEF_TILE_SIZE;
        Constant():
        TITLE("Driveless Car Simulator"),
        SONAR_STD(15.0),
        LAYOUT_DIR("layout"),
        BLOCK_TILE_SIZE(30),
        BELIEF_TILE_SIZE(30)
        {}
    
    };
    struct Config {
        // Maximum depth of the search tree
        int search_depth;
        // Discount factor
        double discount;
        // Random-number seed
        unsigned int root_seed;
        // Amount of CPU time used for search during each move. Does not include the
        // time taken to prune the tree and update the belief.
        double time_per_move;
        // Number of starting states (samples)
        int n_particles;
        // Regularization parameter
        double pruning_constant;
        // Parameter such that eta * width(root) is the target uncertainty at the
        // root of the search tree, used in determining when to terminate a trial.
        double eta;
        // Number of moves to simulate
        int sim_len;
        // Whether the initial upper bound is approximate or true. If approximate,
        // the solver allows initial lower bound > initial upper bound at a node.
        bool approximate_ubound;
        
        Config() :
        search_depth(90),
        discount(0.95),
        root_seed(42),
        time_per_move(1),
        n_particles(500),
        pruning_constant(0),
        eta(0.95),
        sim_len(-1),
        approximate_ubound(false)
        {}
    };
    extern  Constant constant;
    extern Config config;
    
    inline bool Fequals(double a, double b) {
        return fabs(a - b) < TINY;
    }
    
    inline double ExcessUncertainty(
                                    double l, double u, double root_l, double root_u, int depth) {
        return (u-l) // width of current node
        - (config.eta * (root_u-root_l)) // epsilon
        * pow(config.discount, -depth);
    }
    
    inline void ValidateBounds(double& lb, double& ub) {
        if (ub >= lb)
            return;
        if (ub > lb - TINY || config.approximate_ubound)
            ub = lb;
        else
            assert(false);
    }
    
    template<class T>
    inline void hash_combine(size_t& seed, const T& v) {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    
} // namespace

namespace std {
    template<typename S, typename T>
    struct hash<pair<S, T>> {
        inline size_t operator()(const pair<S, T>& v) const {
            size_t seed = 0;
            Globals::hash_combine(seed, v.first);
            Globals::hash_combine(seed, v.second);
            return seed;
        }
    };
    
    template<typename T>
    struct hash<vector<T>> {
        inline size_t operator()(const vector<T>& v) const {
            size_t seed = 0;
            for (const T& ele : v) {
                Globals::hash_combine(seed, ele);
            }
            return seed;
        }
    };
}
namespace Globals {
    Constant constant;
    Config config;
    const double INF = 1e8;
    const double TINY = 1e-8;
} // namespace
#endif
