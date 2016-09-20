//
// Created by juarez on 19/09/16.
//

#include "global.h"

std::vector<DrawObject> drawObjects;
std::vector<DrawObject> objectA, objectB, objectC;

int width = 768;
int height = 768;

double prevMouseX, prevMouseY;
bool mouseLeftPressed;
bool mouseMiddlePressed;
bool mouseRightPressed;
float curr_quat[4];
float prev_quat[4];
float eye[3], lookat[3], up[3];

GLFWwindow *window;

bool wireFrameEnabled = false;
