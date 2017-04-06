////
////  main.cpp
////  TestOpenGL
////
////  Created by HJKBD on 8/20/16.
////  Copyright © 2016 HJKBD. All rights reserved.
////
//
#include <iostream>
#include "header/picojson.h"
#include "header/layout.h"
#include "header/display.h"
#include "header/globals.h"
#include "header/model.h"
#include "header/search.h"
#include "header/inference.h"
#include <cmath>
#include <time.h>
#include <unistd.h>
#include <iomanip>
#include <fstream>
using namespace std;
//#include "header/mdp.h"
#include "KdTree.hpp"
#include "header/decisionmaking2.h"
#include "header/util.h"

GLFWwindow *window = NULL;


Display display = Display();
ofstream myfile;
int main(void) {
    //load the map for running testing examples
    myfile.open ("intention.txt");
    string worldname = "road2";
    Layout layout = Layout(worldname);
    Model model(layout);
    display.setColors(model.getCars());

    
    int SCREEN_WIDTH = layout.getWidth();
    int SCREEN_HEIGHT = layout.getHeight();
    
    Car* mycar = model.getHost();
    vector<Car*> cars = model.getCars();
//    std::cout<<car->isHost()<<std::endl;
    string title = Globals::constant.TITLE;
    begin_graphics(SCREEN_WIDTH, SCREEN_HEIGHT, title);
    std::cout<<typeid(*mycar).name()<<std::endl;
    //loop util the user closes the window
    //bool gameover = false;
    
    bool over = false;
    DecisionAgent2 decision;
    vector<vec2f> mypath;
    
    vector<int> carintentions;
    for (int i = 0; i < model.getOtherCars().size(); i++)
        carintentions.push_back(1);
    bool success = decision.getPath(model, mypath, carintentions);
    vector<vector<vec2f>> mypaths = decision.getPaths();
    std::pair<std::string, vec2f> actionset;
    string filename = "coop";
    
// the change for car is mandatory
    bool change = true;
    srand(time(NULL));
    int i = 0;
    while(!glfwWindowShouldClose(window))
    {
        glClearColor(1.0f, 1.0f, 1.0f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        Display::drawGoal(model.getFinish());
        Display::drawBlocks(model.getBlocks());
        Display::drawLine(model.getLine());
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
                       }
                   }else {
                       car->autonomousAction(mypath, model, NULL);
                       car->update();
                   }
                   drawPolygon(mypath);
               }
               else
               {
                   car->autonomousAction(mypath, model, NULL);
                   car->update();
               }
            }
       }
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
    myfile.close();
    return 1;
}

