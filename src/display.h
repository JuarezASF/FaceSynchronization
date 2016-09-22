//
// Created by juarez on 19/09/16.
//

#ifndef BLENDSHAPESVIEWER_DISPLAY_H
#define BLENDSHAPESVIEWER_DISPLAY_H

#include <algorithm>
#include <cassert>
#include <iostream>

#include <GL/glew.h>


#include <GLFW/glfw3.h>

#define TINYOBJLOADER_IMPLEMENTATION

#include "trackball.h"
#include "global.h"
#include "display.h"
#include "DrawObject.h"

#include <opencv2/highgui.hpp>


#include <sys/time.h>


class timerutil {
public:
    typedef unsigned long int time_t;

    void start() { gettimeofday(tv + 0, &tz); }

    void end() { gettimeofday(tv + 1, &tz); }

    time_t sec() { return (time_t) (tv[1].tv_sec - tv[0].tv_sec); }

    time_t msec() {
        return this->sec() * 1000 +
               (time_t) ((tv[1].tv_usec - tv[0].tv_usec) / 1000);
    }

    time_t usec() {
        return this->sec() * 1000000 + (time_t) (tv[1].tv_usec - tv[0].tv_usec);
    }

    time_t current() {
        struct timeval t;
        gettimeofday(&t, NULL);
        return (time_t) (t.tv_sec * 1000 + t.tv_usec);
    }


private:
    struct timeval tv[2];
    struct timezone tz;
};

void keyboardFunc(GLFWwindow *window, int key, int scancode, int action,
                  int mods);

void clickFunc(GLFWwindow *window, int button, int action, int mods);

void motionFunc(GLFWwindow *window, double mouse_x, double mouse_y);

void Draw(const std::vector<DrawObject> &drawObjects);

void Init();
void CheckErrors(std::string desc);

void CalcNormal(float N[3], float v0[3], float v1[3], float v2[3]);

bool LoadObjAndConvert(float bmin[3], float bmax[3], std::vector<DrawObject> *drawObjects, const char *filename,
                       bool useAsFirst, bool invertNormal);

void reshapeFunc(GLFWwindow *window, int w, int h);

#endif //BLENDSHAPESVIEWER_DISPLAY_H
