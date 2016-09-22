//
// Created by juarez on 19/09/16.
//

#ifndef BLENDSHAPESVIEWER_GLOBAL_H
#define BLENDSHAPESVIEWER_GLOBAL_H

#include <opencv2/highgui.hpp>
#include <tracker/FaceTracker.hpp>

#include "DrawObject.h"
#include "structs.h"

typedef std::vector<DrawObject> Object;

extern Object drawObject;
extern std::vector<Object> keyObjects;

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

extern std::vector<tinyobj::shape_t> firstModelShapes;

extern FACETRACKER::FaceTracker *tracker;
extern FACETRACKER::FaceTrackerParams *params;
extern std::vector<cv::VideoCapture> capture;

#endif //BLENDSHAPESVIEWER_GLOBAL_H
