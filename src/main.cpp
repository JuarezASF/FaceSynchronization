#include <algorithm>
#include <iostream>

#include <GL/glew.h>


#include <GLFW/glfw3.h>

#include "tiny_obj_loader.h"

#include "trackball.h"
#include "global.h"
#include "display.h"

#include <opencv2/highgui.hpp>


#include <sys/time.h>



int alpha_slider0 = 100;
int alpha_slider1 = 0;
int alpha_slider2 = 0;

int alpha_slider_max = 100;

void update();

float weight[] = {1.0f, 0.0f, 0.0f};
float t = 0.0f;


int main(int argc, char **argv) {

    cvNamedWindow("Control", CV_WINDOW_AUTOSIZE);

    cvCreateTrackbar("neutro", "Control", &alpha_slider0, alpha_slider_max, nullptr);
    cvCreateTrackbar("bravo", "Control", &alpha_slider1, alpha_slider_max, nullptr);
    cvCreateTrackbar("feliz", "Control", &alpha_slider2, alpha_slider_max, nullptr);
    cvWaitKey(50);

    Init();


    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return -1;
    }

    window = glfwCreateWindow(width, height, "Obj viewer", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to open GLFW window. " << std::endl;
        glfwTerminate();
        return 1;
    }


    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Callback
    glfwSetWindowSizeCallback(window, reshapeFunc);
    glfwSetKeyCallback(window, keyboardFunc);
    glfwSetMouseButtonCallback(window, clickFunc);
    glfwSetCursorPosCallback(window, motionFunc);

    glewExperimental = true;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW." << std::endl;
        return -1;
    }

    reshapeFunc(window, width, height);

    float bmin[3], bmax[3];
    char initial[] = "obj/rosto_neutro.obj";
    char faceA[] = "obj/rosto_neutro.obj";
    char faceB[] = "obj/rosto_bravo.obj";
    char faceC[] = "obj/rosto_feliz.obj";
    if (!LoadObjAndConvert(bmin, bmax, &drawObjects, initial)) {
        return -1;
    }
    if (!LoadObjAndConvert(bmin, bmax, &objectA, faceA)) {
        return -1;
    }
    if (!LoadObjAndConvert(bmin, bmax, &objectB, faceB)) {
        return -1;
    }
    if (!LoadObjAndConvert(bmin, bmax, &objectC, faceC)) {
        return -1;
    }

    float maxExtent = 0.5f * (bmax[0] - bmin[0]);
    if (maxExtent < 0.5f * (bmax[1] - bmin[1])) {
        maxExtent = 0.5f * (bmax[1] - bmin[1]);
    }
    if (maxExtent < 0.5f * (bmax[2] - bmin[2])) {
        maxExtent = 0.5f * (bmax[2] - bmin[2]);
    }

    while (glfwWindowShouldClose(window) == GL_FALSE) {

        update();
        glfwPollEvents();
        glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_DEPTH_TEST);

        // camera & rotate
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        GLfloat mat[4][4];
        gluLookAt(eye[0], eye[1], eye[2], lookat[0], lookat[1], lookat[2], up[0],
                  up[1], up[2]);
        build_rotmatrix(mat, curr_quat);
        glMultMatrixf(&mat[0][0]);

        // Fit to -1, 1
        glScalef(1.0f / maxExtent, 1.0f / maxExtent, 1.0f / maxExtent);

        // Centerize object.
        glTranslatef(-0.5 * (bmax[0] + bmin[0]), -0.5 * (bmax[1] + bmin[1]),
                     -0.5 * (bmax[2] + bmin[2]));

        Draw(drawObjects);

        glfwSwapBuffers(window);
        cv::waitKey(50);
    }

    glfwTerminate();
}


void update() {

    weight[0] = 1 - alpha_slider1 / 100.0f - alpha_slider2 / 100.0f;
    weight[1] = alpha_slider1 / 100.0f;
    weight[2] = alpha_slider2 / 100.0f;
    float weigthSum = weight[0] + weight[1] + weight[2];


    for (int i = 0; i < drawObjects.size(); i++) {
        for (int j = 0; j < drawObjects[i].attributes.size(); j++) {
            drawObjects[i].attributes[j] = (objectA[i].attributes[j] * weight[0]
                                            + objectB[i].attributes[j] * weight[1]
                                            + objectC[i].attributes[j] * weight[2]) /
                                           weigthSum;
        }
    }
}
