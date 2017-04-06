#ifndef DISPLAY_H
#define DISPLAY_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GLUT/GLUT.h>
#include<cmath>
#include "globals.h"
#include "vec2D.h"
#include "model.h"
#include <algorithm>
#include <iomanip>
#include <cmath>

#define pi 3.1415926
extern void begin_graphics(int SCREEN_WIDTH, int SCREEN_HEIGHT, string title);
using std::vector;

struct Color
{ float r,g,b;
Color(){}
Color(float rr, float gg, float bb):r(rr), g(gg),b(bb){}
void setColor(float rr, float gg, float bb) {r= rr;g=gg;b=bb;}
};

class Display {
private:
    UMAP<size_t,Color> um;
public:
    static const Color GREY;
    static const Color WHITE;
    static const Color RED;
    static const Color GREEN;
    static const Color BLUE;
    static const Color BLACK;
    static const Color YELLOW;
    
    Display(){
        um = UMAP<size_t, Color>();
    }
    
    void setColors(vector<Car*> cars) {
        for (auto car:cars)
            um.insert({(size_t)car, RED});
    }
    void colorchange(const Car* car, const string& color){
        Color col = RED;
        if (color == "green")
            col = GREEN;
        else if(color == "yellow")
            col = YELLOW;
        else if (color == "red")
            col = RED;
        um.at((size_t)car) = col;
    }
    //used to sleep the processor for a while
    static void sleep(float time) {
        long int sleeptime = time*1000000000;
        nanosleep((const struct timespec[]){{0, sleeptime}}, NULL);
    }

    static void drawBlocks(const vector<Block*>& blocks) {
        for(const auto& block: blocks)
          rectangle(block->getCenter(), block->getHeight(), block->getWidth(), Display::GREY,NULL, 1.0);
    }
    
    static void drawGoal(Block& goal) {
      rectangle(goal.getCenter(),goal.getHeight(),goal.getWidth(), Display::GREEN, NULL, 1.0);
    }
    
    static void drawLine(const vector<Line*>&lines)  {
        for(const auto& l :lines)
            line(l->getstart(), l->getend(), Display::BLACK);
    }
    
    void drawCar(Car* car) {
        Color color = um[(size_t)car];
        if (car->isHost()) color = Display::BLUE;
        Vector2f dir = car->getDir();
        rectangle(car->getPos(), Car::WIDTH, Car::LENGTH, color, &dir);
    }
    
    void drawCars(vector<Car*>& cars) {
        Color color = Display::BLUE;
        for (Car*car: cars) {
         Vector2f dir = car->getDir();
         rectangle(car->getPos(), Car::WIDTH, Car::LENGTH, color, &dir);
        }
    }
    
    void drawOtherCar(const vector<Car*>& cars){
        for (Car* car:cars) drawCar(car);
    }
    
    static void drawGraph(vector<Block>& graph){
        for(Block block : graph)
            rectangle(block.getCenter(), block.getHeight(), block.getWidth(), Display::BLUE,NULL, 1.0);
    }
    
    static void drawTriagnle(vector<Vector2f>& triangles) {
        triangle(triangles, Display::YELLOW, 1);
    }
    
    //draw the rectangle
    static void rectangle(Vector2f pos, int length, int width, Color color, Vector2f* dir = NULL, int filled = 1, int thickness = 0) {
        
        Vector2f vertices[] = {
            Vector2f(-width/2.0, -length/2.0),
            Vector2f(+width/2.0, -length/2.0),
            Vector2f(+width/2.0, +length/2.0),
            Vector2f(-width/2.0, +length/2.0)};
        
        float angle = 0;
        
        if (dir){
            dir->normalized();
            angle = -dir->get_angle_between(Vector2f(1, 0));
        }
        
        float polygonvertices[8];
        for (int i = 0; i < 4; i++)
        {
            vertices[i].rotate(angle);
            polygonvertices[2*i] = pos[0] + vertices[i][0];
            polygonvertices[2*i+1] = pos[1] + vertices[i][1];
        }
        
        glPushAttrib( GL_POLYGON_BIT);
        glPushAttrib(GL_LINE_BIT);
        if (thickness!=0) glLineWidth(thickness);
        if (filled == 0) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glColor3f(color.r, color.g, color.b);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, polygonvertices);
        glDrawArrays(GL_POLYGON, 0, 4);
        glDisableClientState(GL_VERTEX_ARRAY);
        glPopAttrib();
        
    }
    
    static void triangle(vector<Vector2f> pos, Color color, int filled = 1, int thickness = 0) {
        

        float polygonvertices[8];
        for (int i = 0; i < 3; i++)
        {
            polygonvertices[2*i] = pos[i].x;
            polygonvertices[2*i+1] = pos[i].y;
        }
        
        glPushAttrib( GL_POLYGON_BIT);
        glPushAttrib(GL_LINE_BIT);
        if (thickness!=0) glLineWidth(thickness);
        if (filled == 0) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glColor3f(color.r, color.g, color.b);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, polygonvertices);
        glDrawArrays(GL_POLYGON, 0, 3);
        glDisableClientState(GL_VERTEX_ARRAY);
        glPopAttrib();
        
    }
    
    
    static void line(Vector2f here, Vector2f there, const Color& color, int width=1, pii dash = pii(4,4)){
        GLfloat polygonvertices[4] = {
              here[0], here[1], there[0], there[1]
        };
        glColor3f(color.r, color.g, color.b);
        glEnable(GL_LINE_STIPPLE);
        glLineWidth(3);
        glLineStipple(3, 0x0C0F);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, polygonvertices);
        glDrawArrays(GL_LINES, 0, 2);
        glDisable(GL_LINE_STIPPLE);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
    
    static void square(float x, float y, float grid_size, const Color& color, bool filled = 0, int width = 3) {
        Vector2f pos(x, y);
        rectangle(pos, grid_size, grid_size, color, NULL, filled, width);
    }
    static void text(float x, float y, const Color& color, const std::string& str, bool isIteration) {
 
        glColor3f(color.r, color.g, color.b);
        void* font = NULL;
        if (isIteration) font = GLUT_BITMAP_TIMES_ROMAN_24;
        else font =GLUT_BITMAP_8_BY_13;
        
        glRasterPos2f(x, y);
        for (int i= 0; i < str.length(); i++) {
            glutBitmapCharacter(font, str.data()[i]);
        }
        
    }
    
    //draw the plolygon shape
    static void drawPolygon(const vector<Vector2f>&vertices, const Color& color){
        float polygonvertices[6];
        for (int i = 0; i < 3; i++)
        {
            polygonvertices[2*i] =  vertices[i][0];
            polygonvertices[2*i+1] = vertices[i][1];
        }
        glColor3f(color.r, color.g, color.b);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, polygonvertices);
        glDrawArrays(GL_POLYGON, 0, 3);
        glDisableClientState(GL_VERTEX_ARRAY);
        
    }
    
    static void Circle(float x, float y, float radius, const Color& color) {
        int i;
        int triangleAmount = 40; //# of triangles used to draw circle
        
        GLfloat twicePi = 2.0f * pi;
        glBegin(GL_TRIANGLE_FAN);
        glColor3f(color.r, color.g, color.b);
        glVertex2f(x, y); // center of circle
        for(i = 0; i <= triangleAmount;i++) {
            glVertex2f(
                       x + (radius * cos(i *  twicePi / triangleAmount)),
                       y + (radius * sin(i * twicePi / triangleAmount))
                       );
        }
        glEnd();
        
    }
};
const Color Display::GREY = {0.5f, 0.5f, 0.5f};
const Color Display::WHITE = {1.0f, 1.0f, 1.0f};
const Color Display::RED = {1.0f, 0.0f, 0.0f};
const Color Display::GREEN = {0.16f, 0.49f, 0.31f};
const Color Display::BLUE = {0.0f, 0.0f, 1.0f};
const Color Display::BLACK = {0.0f, 0.0f, 0.0f};
const Color Display::YELLOW = {0.0f, 1.0f, 1.0f};

#endif
