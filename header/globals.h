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
//    const double INF = 1e8;
//    const double TINY = 1e-8;
} // namespace
#endif
