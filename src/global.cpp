//
// Created by juarez on 19/09/16.
//

#include "global.h"

Object drawObject;
std::vector<Object> keyObjects;
std::vector<tinyobj::shape_t> firstModelShapes;

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

