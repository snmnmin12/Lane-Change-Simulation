////  Created by HJKBD on 8/20/16.
////  Copyright © 2016 HJKBD. All rights reserved.
/*
This is the main page to start the programs
*/
#include <iostream>
#include "header/picojson.h"
#include "header/layout.h"
#include "header/display.h"
#include "header/car.h"
#include "header/globals.h"
#include "header/model.h"
#include "header/search.h"
#include "header/inference.h"
#include <cmath>
#include <time.h>
#include <unistd.h>
#include <iomanip>
using namespace std;
#include "KdTree.hpp"
#include "header/decisionmaking2.h"

GLFWwindow *window = NULL;
void begin_graphics(int SCREEN_WIDTH, int SCREEN_HEIGHT, string title);
vector<int> infer(const Model& model);

Display display = Display();
bool gameover(Model& model){
    if (model.checkVictory()
        ||model.checkCollision(model.getHost()))
        return true;
    return false;
}

void drawPolygon(vector<Vector2f>& polygonvertices){
    glPushAttrib( GL_POLYGON_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(1.0f, 0.0f, 0.0f);
    float* vertices = new float[polygonvertices.size()*2];
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < polygonvertices.size();i++) {
        vertices[2*i] = polygonvertices[i][0];
        vertices[2*i+1] = polygonvertices[i][1];
        glVertex2d(vertices[2*i], vertices[2*i+1]);
    }
    glEnd();
    glPopAttrib();
    delete[] vertices;
}

int main(void) {
    //load the map for running testing examples

    string worldname = "road2";
    Layout layout = Layout(worldname);
    Model model(layout);
    display.setColors(model.getCars());
    SEARCH::Search search(&model, model.getFinish().getCenter());
    vector<Vector2f> path;
    path = search.path();
    
   // tree construction
    vector<point<float> > pointlist;
    for (int i = 0; i < path.size(); i++)
        pointlist.push_back(point<float>(path[i][0], path[i][1], i));
    
    kdtree::kdtree<point<float>> *tree = nullptr;
    tree = new kdtree::kdtree<point<float>>(pointlist);
    
    int SCREEN_WIDTH = layout.getWidth();
    int SCREEN_HEIGHT = layout.getHeight();
    
    Car* mycar = model.getHost();
    vector<Car*> cars = model.getCars();
//    std::cout<<car->isHost()<<std::endl;
    string title = Globals::constant.TITLE;
    begin_graphics(SCREEN_WIDTH, SCREEN_HEIGHT, title);
    std::cout<<typeid(*mycar).name()<<std::endl;
    
    bool over = false;
    DecisionAgent2 decision;
    vector<vec2f> mypath;
    vector<int> carintentions;
    for (int i = 0; i < model.getOtherCars().size(); i++)
        carintentions.push_back(1);
    bool success = decision.getPath(model, mypath, carintentions);
    vector<vector<vec2f>> mypaths = decision.getPaths();
    std::pair<std::string, vec2f> actionset;
// the change for car is mandatory
    bool change = true;
    srand(time(NULL));
    while(!glfwWindowShouldClose(window))
    {
        glClearColor(1.0f, 1.0f, 1.0f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        Display::drawGoal(model.getFinish());
        Display::drawBlocks(model.getBlocks());
        Display::drawLine(model.getLine());
        //drawPolygon(path);
        for(auto p:mypaths)
            drawPolygon(p);
        display.drawCar(model.getHost());
        display.drawOtherCar(model.getOtherCars());
        
        if (!gameover(model)) {
            //update cars here for further processing
           for (Car* car: cars) {
               if (car == mycar) {
                   if (mypath.size() ==0 || abs(mycar->getPos().x-mypath[mypath.size()-1].x)<10)
                   {
                       success = decision.getPath(model, mypath, carintentions);
                       change = decision.isChangeRequired(car, model);
                       mypaths = decision.getPaths();
                       if (!success && change) {
                           carintentions = infer(model);
                           mypath.clear();
                           decision.ApplyAction(model, 0, "dec");
                       }else {
                           car->autonomousAction(mypath, model, NULL);
                           car->update();
                           //change = decision.isChangeRequired(car, model);
                       }
                       //drawPolygon(mypath);
                   }else {
                       car->autonomousAction(mypath, model, NULL);
                       car->update();
                   }
                   drawPolygon(mypath);
               }
               else
               {
                   //infer(model);
                   car->autonomousAction(mypath, model, NULL);
                   car->update();
               }
            }
       }
        //paths = decision.generatePaths(model);
        glfwSwapBuffers(window);
        glfwPollEvents();
        over = (gameover(model)||glfwWindowShouldClose(window));
        Display::sleep(0.05);
     
    }
    if (model.checkVictory())
        std::cout<<"The car win"<<endl;
    else
        std::cout<<"You lose the car game" <<endl;
    glfwTerminate();
    
    if (!tree)
        delete tree;
    return 1;
}


void observe(Host* car, const Model& model) {
    car->makeObse(model);
    vector<Car*>cars = model.getOtherCars();
    for (int index = 0; index < cars.size(); index++) {
        Agent* car = dynamic_cast<Agent*>(cars[index]);
        int i = model.toindex(car);
        Inference::MarginalInference* inference = car->getInference(i+1, model);
        inference->observe(model);
    }
}

vector<int> infer(const Model& model) {
    Host* car = dynamic_cast<Host*>(model.getHost());
    vector<int> cartoIntention;
    observe(car, model);
    vector<string> colors{"green","red"};
    vector<Car*> cars = model.getOtherCars();
    for (int k = 0; k < cars.size(); k++) {
        Agent* car = dynamic_cast<Agent*>(cars[k]);
        int index = model.toindex(car);
        vector<float>belief = car->getInference(index+1, model)->getBelief();
        int maxindex = 0;
        for (int i = 0; i < belief.size(); i++) {
            if (belief[i]>belief[maxindex]) {
                maxindex = i;
            }
        }
        cartoIntention.push_back(maxindex);
        display.colorchange(car,colors[maxindex]);
    }
    return cartoIntention;
}


//begin window;
void begin_graphics(int SCREEN_WIDTH, int SCREEN_HEIGHT, string title) {
    if (!glfwInit()){
        return;
    }
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    //create a window mode window and its openGL context
    window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, title.c_str(), NULL, NULL);
    if (!window) {
        glfwTerminate();
        return ;
    }
    glfwMakeContextCurrent(window);
    glOrtho(0, SCREEN_WIDTH, 0, SCREEN_HEIGHT, 0, 1);
}
