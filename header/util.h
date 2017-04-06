
#ifndef UTIL_H
#define UTIL_H
/*
 This file is for any helper function not listed in other files, will be here
 */
extern GLFWwindow *window;
extern Display display;
//initiation to start the open gl program
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

//to check if the game is over
bool gameover(Model& model){
    if (model.checkVictory()
        ||model.checkCollision(model.getHost()))
        return true;
    return false;
}
//draw lines 
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
    //    glEnableClientState(GL_VERTEX_ARRAY);
    //    glVertexPointer(2, GL_FLOAT, 0, vertices);
    //    glDrawArrays(GL_POLYGON, 0, polygonvertices.size());
    //    glDisableClientState(GL_VERTEX_ARRAY);
    delete[] vertices;
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
    //beliefs = []
    vector<string> colors{"green","red"};
    vector<Car*> cars = model.getOtherCars();
    for (int k = 0; k < cars.size(); k++) {
        Agent* car = dynamic_cast<Agent*>(cars[k]);
        int index = model.toindex(car);
        vector<float>belief = car->getInference(index+1, model)->getBelief();
        //        if (k != index)
        //            cout<<"I am here"<<endl;
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

#endif
