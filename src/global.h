//
// Created by juarez on 19/09/16.
//

#ifndef BLENDSHAPESVIEWER_GLOBAL_H
#define BLENDSHAPESVIEWER_GLOBAL_H

#include "DrawObject.h"

extern std::vector<DrawObject> drawObjects;
extern std::vector<DrawObject> objectA, objectB, objectC;

extern int width;
extern int height;

extern double prevMouseX, prevMouseY;
extern bool mouseLeftPressed;
extern bool mouseMiddlePressed;
extern bool mouseRightPressed;
extern float curr_quat[4];
extern float prev_quat[4];
extern float eye[3], lookat[3], up[3];

extern GLFWwindow *window;

extern bool wireFrameEnabled;
#endif //BLENDSHAPESVIEWER_GLOBAL_H
