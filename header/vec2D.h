//
//  vec2D.hpp
//  CarGame
//
//  Created by HJKBD on 8/20/16.
//  Copyright © 2016 HJKBD. All rights reserved.
//

#ifndef vec2D_h
#define vec2D_h

#include <cfloat>
#include <climits>
#include <tuple>
#include "globals.h"
#include <iostream>
using std::ostream;
/*The Vector2d class is an object consisting of simply an x and
 y value. Certain operators are overloaded to make it easier
 for vector math to be performed.*/
#define PI 3.14159265

template<class T>
class Vector2d {
public:
    /*The x and y values are public to give easier access for
     outside funtions. Accessors and mutators are not really
     necessary*/
    T x;
    T y;
    //Constructor assigns the inputs to x and y.
    Vector2d(): x(T(0)), y(T(0)) {}
    Vector2d(const T& vx, const T& vy): x(vx), y(vy) {}
    Vector2d(const Vector2d& v):x(v[0]), y(v[1]){}
    T& operator[](int i) { return (i == 0)?x:y; }
    T operator[](int i) const { return (i== 0)?x:y;}
    /*The following operators simply return Vector2ds that
     have operations performed on the relative (x, y) values*/
    Vector2d& operator+=(const Vector2d& v) { x += v.x; y += v.y; return *this; }
    Vector2d& operator-=(const Vector2d& v) { x -= v.x; y -= v.y; return *this; }
    Vector2d& operator*=(const Vector2d& v) { x *= v.x; y *= v.y; return *this; }
    Vector2d& operator/=(const Vector2d& v) { x /= v.x; y /= v.y; return *this; }
    //Check if the Vectors have the same values (uses pairwise comparison of `std::tuple` on the x,y values of L and R.
    friend bool operator==(const Vector2d& L, const Vector2d& R) { return std::tie(L.x, L.y) == std::tie(R.x, R.y); }
    friend bool operator!=(const Vector2d& L, const Vector2d& R) { return !(L == R); }
    //Check if the Vectors have the same values (uses pairwise comparison of `std::tuple` on the x,y values of L and R.
    friend bool operator< (const Vector2d& L, const Vector2d& R) { return std::tie(L.x, L.y) < std::tie(R.x, R.y); }
    friend bool operator>=(const Vector2d& L, const Vector2d& R) { return !(L < R); }
    friend bool operator> (const Vector2d& L, const Vector2d& R) { return   R < L ; }
    friend bool operator<=(const Vector2d& L, const Vector2d& R) { return !(R < L); }
    //Negate both the x and y values.
    Vector2d operator-() const { return Vector2d(-x, -y); }
    //Apply scalar operations.
    Vector2d& operator*=(const T& s) { x *= s; y *= s; return *this; }
    Vector2d& operator/=(const T& s) { x /= s; y /= s; return *this; }
    
    //utility functions below for analaysis
    
    //roate
    void rotate(float angle_degrees);
    //return roated vector
    
    Vector2d<T> rotated(float angle_degrees);
    //Product functions
    T dot(const Vector2d<T>&, const Vector2d<T>&);
    T cross(const Vector2d<T>&, const Vector2d<T>&);
    //Returns the length of the vector from the origin.
    T Length(const Vector2d<T>& v);
    //set length of the vector
    T Length() { return sqrt((this->x * this->x) + (this->y * this->y));}
    
    //get distance between objects
    T get_distance(const Vector2d<T>& other) {
        return sqrt(pow((x - other[0]),2) + pow((y - other[1]),2));}
    //nomalized vector
    void normalized();
    Vector2d<T> normalized(const Vector2d<T>& v);
    
    //Return a vector perpendicular to the left.
    Vector2d<T> perpendicular() { return Vector2d<T>(y, -x);}
    
    //Return true if two line segments intersect.
    bool Intersect(const Vector2d<T>&, const Vector2d<T>&, const Vector2d<T>&, const Vector2d<T>&);
    
    //Return the point where two lines intersect.
    Vector2d<T> get_Intersect(const Vector2d<T>&, const Vector2d<T>&, const Vector2d<T>&, const Vector2d<T>&);
    
    //return the angle
    float get_angle() {
        if (this->Length() == 0) return 0;
        float angle =  atan2(y, x);
        return angle/PI*180;
        ;
    }
    //return the angle between two vectors
    float get_angle_between(const Vector2d<T>&);
    
    // get the opposite direction vector
    Vector2d<T> get_reflection() {
        return Vector2d(-x, -y);
    }
    
   //projection
    T project(const Vector2d<T>& point, const Vector2d<T>& vector);

    // projection
    std::pair<T,T> projectPoints(const std::vector<Vector2d<T>>& points, const Vector2d<T>& vec);
    
    friend ostream& operator<<(std::ostream& os, const Vector2d<T>& v)
    {
        os << "("<<v[0] << ", " << v[1] << ")";
        return os;
    }
};


template<class T>
Vector2d<T> operator+(const Vector2d<T>& L, const Vector2d<T>& R) {
    return Vector2d<T>(L) += R; }

template<class T>
Vector2d<T> operator-(const Vector2d<T>& L, const Vector2d<T>& R) {
    return Vector2d<T>(L)-= R; }

template<class T>
Vector2d<T> operator*(const Vector2d<T>& L, const Vector2d<T>& R) {
    return Vector2d<T>(L)*= R; }

template<class T>
Vector2d<T> operator/(const Vector2d<T>& L, const Vector2d<T>& R) {
    return Vector2d<T>(L)/= R; }

template<class T>
Vector2d<T> operator*(const T& s, const Vector2d<T>& v)
{ return Vector2d<T>(v) *= s; }

template<class T>
Vector2d<T> operator*(const Vector2d<T>& v, const T& s)
{ return Vector2d<T>(v) *= s; }

template<class T>
Vector2d<T> operator/(const T& s, const Vector2d<T>& v) {
    return Vector2d<T>(v) /= s;
  }

template<class T>
Vector2d<T> operator/(const Vector2d<T>& v, const T& s) {
    return Vector2d<T>(v) /= s;
  }

template<class T>
T dot(const Vector2d<T>& a, const Vector2d<T>& b){
    return ((a.x * b.x) + (a.y * b.y));}

template<class T>
T cross(const Vector2d<T>& a, const Vector2d<T>& b){
    return ((a.x * b.y) - (a.y * b.x));
}
template<class T>
T Length(const Vector2d<T>& v) { return sqrt((v.x * v.x) + (v.y * v.y));}

template<class T> Vector2d<T> normalized(const Vector2d<T>& v){
    T  magnitude = Length(v);
    return Vector2d<T>(v.x / magnitude, v.y / magnitude);
}

template<class T> void Vector2d<T>::normalized(){
    T magnitude = this->Length();
    if (magnitude != 0) {
    x /= magnitude;
    y /= magnitude;
    }else{
        x = 0;
        y = 0;
    }
}

template<class T>
bool Vector2d<T>::Intersect(const Vector2d<T>&aa, const Vector2d<T>&ab, const Vector2d<T>&ba, const Vector2d<T>&bb){
    Vector2d<T> p = aa;
    Vector2d<T> r = ab - aa;
    Vector2d<T> q = ba;
    Vector2d<T> s = bb - ba;
    
    float t = CrossProduct((q - p), s) / CrossProduct(r, s);
    float u = CrossProduct((q - p), r) / CrossProduct(r, s);
    
    return (0.0 <= t && t <= 1.0) &&
    (0.0 <= u && u <= 1.0);
}

template<class T>
Vector2d<T> Vector2d<T>::get_Intersect(const Vector2d<T>&aa, const Vector2d<T>&ab, const Vector2d<T>&ba, const Vector2d<T>&bb){
    T pX = (aa.x*ab.y - aa.y*ab.x)*(ba.x - bb.x) - (ba.x*bb.y - ba.y*bb.x)*(aa.x - ab.x);
    T pY = (aa.x*ab.y - aa.y*ab.x)*(ba.y - bb.y) -(ba.x*bb.y - ba.y*bb.x)*(aa.y - ab.y);
    T denominator = (aa.x - ab.x)*(ba.y - bb.y) -
    (aa.y - ab.y)*(ba.x - bb.x);
    return Vector2d<T>(pX / denominator, pY / denominator);
}

template<class T>
void Vector2d<T>::rotate(float angle_degrees){
    T radians = angle_degrees/180*PI;
    T co = cos(radians);
    T sn = sin(radians);
    T xx = x*co - y*sn;
    T yy = x*sn + y*co;
    x = xx;
    y = yy;
}
template<class T>
Vector2d<T> Vector2d<T>::rotated(float angle_degrees){
    float radians = angle_degrees/180*PI;
    float co = cos(radians);
    float sn = sin(radians);
    T xx = x*co - y*sn;
    T yy = x*sn + y*co;
    return Vector2d(xx, yy);
}
template<class T>
float Vector2d<T>::get_angle_between(const Vector2d<T>& other)
{
    T cross = x*other[1] - y*other[0];
    T dot = x*other[0] + y*other[1];
    return atan2(cross, dot)/PI*180;
}

template<class T>
T project(const Vector2d<T>& point, const Vector2d<T>& vector)
{ return dot(point,vector) / dot(vector, vector); }

template<class T>
std::pair<T,T> projectPoints(const std::vector<Vector2d<T>>& points, const Vector2d<T>& vec)
{
    std::vector<T> values;
    for (const auto& point: points) {
        T value = project(point, vec);
        values.push_back(value);
    }
    auto ele = std::minmax_element(values.begin(), values.end());
   // std::cout<<*ele.first<<std::endl;
   // std::pair<T,T> result = make_pair(*ele.first, *ele.second);
    std::pair<T,T> result(*ele.first, *ele.second);//(points[0], points[1]);
    return result;
    //return ele;
}
#endif /* vec2D_hpp */
