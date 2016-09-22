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

#define HIGHRES 0

#if HIGHRES > 0
int quantityOfPoses = 1;
std::vector<std::string> keyPoseFiles = {
        "obj/Rosto_HD.OBJ",
};

std::vector<std::string> keyPoseNames = {
        "neutro",
};

#else
int quantityOfPoses = 14;
std::vector<std::string> keyPoseFiles = {
        "obj/rosto_neutro.obj",
        "obj/rosto_bravo.obj",
        "obj/rosto_bravo.obj",
        "obj/rosto_bravo.obj",
        "obj/rosto_feliz.obj",
        "obj/rosto_cheek_L.OBJ",
        "obj/rosto_cheek_R.OBJ",
        "obj/rosto_Eyebrow_L.OBJ",
        "obj/rosto_Eyebrow_R.OBJ",
        "obj/rosto_Eye_L.OBJ",
        "obj/rosto_Eye_R.OBJ",
        "obj/rosto_Mouth_Open.OBJ",
        "obj/rosto_Mouth_DuckFace.OBJ",
        "obj/rosto_Nariz_flare.OBJ"
};

std::vector<std::string> keyPoseNames = {
        "neutro",
        "bravo",
        "bravo2",
        "bravo3",
        "feliz",
        "left cheek",
        "right cheek",
        "left eyebrow",
        "right eyebrow",
        "left eye",
        "right eye",
        "boca aberta",
        "boca fechada",
        "nariz",
};

#endif

int *sliders = new int[quantityOfPoses];
float *weight = new float[quantityOfPoses];

int alpha_slider_max = 100;

void update();

float t = 0.0f;


int main(int argc, char **argv) {

    //zero out weights
    for (int k = 0; k < quantityOfPoses; k++) {
        weight[k] = 0.0;
        sliders[k] = (int) 0;
    }

    //initialize open cv
    cvNamedWindow("Control", CV_WINDOW_AUTOSIZE);

    int sliderIdx = 0;
    for (auto poseName : keyPoseNames) {
        cvCreateTrackbar(poseName.c_str(), "Control", sliders + (sliderIdx++), alpha_slider_max, nullptr);
    }

    //very important to give it some time to start!
    cvWaitKey(100);

    //initialize open gl
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

    glewExperimental = (GLboolean) true;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW." << std::endl;
        return -1;
    }

    reshapeFunc(window, width, height);

    float bmin[3], bmax[3];
    if (!LoadObjAndConvert(bmin, bmax, &drawObject, keyPoseFiles[0].c_str(), true)) {
        return -1;
    }
    for (auto poseFile : keyPoseFiles) {
        keyObjects.emplace_back();
        if (!LoadObjAndConvert(bmin, bmax, &keyObjects[keyObjects.size() - 1], poseFile.c_str(), false)) {
            return -1;
        }
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
        gluLookAt(eye[0], eye[1], eye[2], lookat[0], lookat[1], lookat[2], up[0], up[1], up[2]);
        build_rotmatrix(mat, curr_quat);
        glMultMatrixf(&mat[0][0]);

        // Fit to -1, 1
        glScalef(1.0f / maxExtent, 1.0f / maxExtent, 1.0f / maxExtent);

        // Centerize object.
        glTranslatef((GLfloat) (-0.5 * (bmax[0] + bmin[0])), (GLfloat) (-0.5 * (bmax[1] + bmin[1])),
                     (GLfloat) (-0.5 * (bmax[2] + bmin[2])));

        Draw(drawObject);

        glfwSwapBuffers(window);
        cv::waitKey(50);
    }

    glfwTerminate();
}


void update() {

    float weightSum = 0;
    for (int i = 1; i < quantityOfPoses; i++) {
        weight[i] = ((float) sliders[i]) / alpha_slider_max;
        weightSum += weight[i];
    }

    weight[0] = 1.0f - weightSum;


    for (int i = 0; i < drawObject.size(); i++) {
        for (int j = 0; j < drawObject[i].attributes.size(); j++) {
            drawObject[i].attributes[j] = 0.0;
            for (int k = 0; k < quantityOfPoses; k++) {
                drawObject[i].attributes[j] += weight[k] * keyObjects[k][i].attributes[j];
            }
        }
    }
}
