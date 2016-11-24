#include <algorithm>
#include <iostream>

#include <GL/glew.h>


#include <GLFW/glfw3.h>

#include "tiny_obj_loader.h"

#include "trackball.h"
#include "global.h"
#include "display.h"


#define HIGHRES 0

#if HIGHRES > 0
int QUANTITY_POSES = 10;
int quantityOfPoses = 10;

std::vector<std::string> keyPoseFiles = {
        "obj/HD/rosto_normal.OBJ",
        "obj/HD/rosto_feliz.OBJ",
        "obj/HD/rosto_bravo.OBJ",
        "obj/HD/HD_eye_L.OBJ",
        "obj/HD/HD_eye_R.OBJ",
        "obj/HD/HD_eye_L+.OBJ",
        "obj/HD/HD_eye_R+.OBJ",
        "obj/HD/HD_rosto_aberto.OBJ",
        "obj/HD/HD_rosto_duck.OBJ",
        "obj/HD/HD_rosto_surpreso.OBJ",
};

std::vector<std::string> keyPoseNames = {
        "neutro",
        "feliz",
        "bravo",
        "eyeL",
        "eyeR",
        "eyeL+",
        "eyeR+",
        "aberto",
        "duck",
        "surpreso",
};

#else
int quantityOfPoses = 12;


std::vector<std::string> keyPoseFiles = {
        "obj/rosto_neutro.obj",
        "obj/rosto_feliz.obj",
        "obj/rosto_bravo.obj",
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
        "feliz",
        "bravo",
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


int quantityOfSensors = quantityOfPoses;

int *sliders = new int[quantityOfPoses];
float *weight = new float[quantityOfPoses];
double *sensors = new double[quantityOfSensors];

int alpha_slider_max = 150;

void update();

void updateSensors(std::vector<cv::Point3d> pointsFace);

void updateTracking();

float t = 0.0f;

//frame is used for web cam input; but the CSIRO face detection works only on gray scale images
cv::Mat frame, grayScale;
std::vector<cv::Point3d> points3d(66), points3dOld(66), points3dDiff(66);

double capHeight;

void onSaveImageCallBack(int k);

int main(int argc, char **argv) {


    //initialize open cv windows for slide trackers
    cvNamedWindow("Control0", CV_WINDOW_NORMAL);
    cvNamedWindow("Control1", CV_WINDOW_KEEPRATIO);

    //initialize sliders
    int qtdPerControlWindow = 8;
    int sliderIdx = 0;
    for (auto poseName : keyPoseNames) {
        std::string windowName = "Control" + std::to_string((int) (sliderIdx * 1.0 / qtdPerControlWindow));
        cvCreateTrackbar(poseName.c_str(), windowName.c_str(), sliders + (sliderIdx++), alpha_slider_max, nullptr);
        cvSetTrackbarPos(poseName.c_str(), windowName.c_str(), 0);
    }

    for(int i = 0; i < quantityOfPoses; i++){
        sliders[i] = 0;

    }

    cvCreateTrackbar("save_img", "Control1", nullptr, 1, onSaveImageCallBack);

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

    //load OBJ's while keeping track of maximum co-ordenades required to display objects
    float bmin[3], bmax[3];

    //first object on array is assumed to be neutral pose
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

    /////////////////////////////////

    while (glfwWindowShouldClose(window) == GL_FALSE) {
        updateSensors(points3d);

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
    for (int i = 1; i < quantityOfSensors; i++) {
        float measurement;

        measurement = (float) sensors[i];

        weight[i] = measurement / 100.0f;
        weightSum += weight[i];
    }

    weight[0] = 1.0f - weightSum;


    for (int i = 0; i < drawObject.size(); i++) {
        for (int j = 0; j < drawObject[i].attributes.size(); j++) {
            drawObject[i].attributes[j] = 0.0;
            for (int k = 0; k < quantityOfSensors; k++) {
                drawObject[i].attributes[j] += weight[k] * keyObjects[k][i].attributes[j];
            }
        }
    }
}

void updateSensors(std::vector<cv::Point3d> pointsFace) {
    //default behavior for sensors is to follow tracker
    for (int i = 0; i < quantityOfPoses; i++)
        sensors[i] = sliders[i];

}


int savedImageCounter = 0;
time_t timev;
void onSaveImageCallBack(int k){

    time(&timev);

//    cv::imwrite("img/savedImg_" + std::to_string(timev) + ".jpg", ADJ_ESTERROR)
    cvSetTrackbarPos("save_img", "Control1", 0);
}

