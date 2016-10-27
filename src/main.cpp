#include <algorithm>
#include <iostream>

#include <GL/glew.h>


#include <GLFW/glfw3.h>

#include "tiny_obj_loader.h"

#include "trackball.h"
#include "global.h"
#include "display.h"
#include "util.h"
#include "Filter.h"

#include <opencv2/highgui.hpp>

#include <tracker/FaceTracker.hpp>


#include <sys/time.h>

#define HIGHRES 0

#if HIGHRES > 0
int quantityOfPoses = 8;
std::vector<std::string> keyPoseFiles = {
        "obj/Rosto_HD.OBJ",
        "obj/Rosto_HD.OBJ",
        "obj/Rosto_HD.OBJ",
        "obj/Rosto_HD.OBJ",
        "obj/Rosto_HD.OBJ",
        "obj/Rosto_HD.OBJ",
        "obj/Rosto_HD.OBJ",
        "obj/Rosto_HD.OBJ",
};

std::vector<std::string> keyPoseNames = {
        "neutro",
        "neutro2",
        "neutro3",
        "neutro4",
        "neutro5",
        "neutro6",
        "neutro7",
        "neutro8",
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


bool useFilter = true;
std::vector<HanningFilter> filters(quantityOfSensors);

int alpha_slider_max = 150;

void update();

void updateSensors(std::vector<cv::Point3d> pointsFace);

void updateTracking();

float t = 0.0f;

//frame is used for web cam input; but the CSIRO face detection works only on gray scale images
cv::Mat frame, grayScale;
std::vector<cv::Point3d> points3d(66), points3dOld(66), points3dDiff(66);
std::vector<cv::Point_<double> > pointsCam1, points;

//camera parameters
double f1 = (854.792781659906610 + 857.614193372593600) / 2;
double f2 = (834.378121568220080 + 835.838850269775660) / 2;
double b = 18;

double capHeight;



int main(int argc, char **argv) {



    if (!startFaceTracker()) {
        return 0;
    };
    if (!startCapture()) {
        return 0;
    };

    //zero out weights
    for (int k = 0; k < quantityOfPoses; k++) {
        weight[k] = 0.0;
        sliders[k] = (int) 0;
    }

    //initialize open cv windows for slide trackers
    cvNamedWindow("Control0", CV_WINDOW_NORMAL);
    cvNamedWindow("Control1", CV_WINDOW_KEEPRATIO);
    cvNamedWindow("Input#1", CV_WINDOW_KEEPRATIO);

    //initialize sliders
    int qtdPerControlWindow = 8;
    int sliderIdx = 0;
    for (auto poseName : keyPoseNames) {
        std::string windowName = "Control" + std::to_string((int) (sliderIdx * 1.0 / qtdPerControlWindow));
        cvCreateTrackbar(poseName.c_str(), windowName.c_str(), sliders + (sliderIdx++), alpha_slider_max, nullptr);
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
    filters[8].updateP(4.0f, 1.0f, 1.0f, 0.16f);
    filters[8].updateP(4.0f, 1.0f, 1.0f, 0.16f);

    /////////////////////////////////

    while (glfwWindowShouldClose(window) == GL_FALSE) {
        updateTracking();

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

void updateTracking() {
    int detectionQuality;
    int cameraNum = 1;

    for (auto cap : capture) {

        cap >> frame;
        if(frame.empty()){
            std::cout << "end of video reached!" << std::endl;
        }

        cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);

        //update state of tracker with new frame
        detectionQuality = tracker->NewFrame(grayScale, params);

        if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
            tracker->Reset();
        }

        //obtain points that were tracked
        points = tracker->getShape();

        int count = 0;

        //draw point on input frame
        for (auto p : points) {
//            putText(frame, std::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
            count++;
            cv::circle(frame, p, 1, cv::Scalar(255, 0, 0));

        }

        points3dOld = points3d;


        if (cameraNum == 1){
            pointsCam1 = points;
            imshow("Input#" + std::to_string(cameraNum), frame);

        }
        else if (detectionQuality) {
            capHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
            for (int i = 0; i < points.size(); i++) {
                points[i].y = cap.get(CV_CAP_PROP_FRAME_HEIGHT) - points[i].y;
                pointsCam1[i].y = cap.get(CV_CAP_PROP_FRAME_HEIGHT) - pointsCam1[i].y;

                points3d[i].z = f1 * f2 * b / (pointsCam1[i].x * f2 - points[i].x * f1);
                points3d[i].x = (pointsCam1[i].x / f1 + points[i].x / f2) * points3d[i].z / 2;
                points3d[i].y = (pointsCam1[i].y / f1 + points[i].y / f2) * points3d[i].z / 2;
            }
        }

//            if(!points3dOld.empty())
//                for(int i = 0; i < points3d.size(); i++)
//                {
//                    points3dDiff[i] = points3d[i] - points3dOld[i];
//                }

        std::cout << "diff " << points3d[54].x - points3d[48].x << std::endl;
        // Z = f1*f2*b/(x1*f2 - x2*f1);

        cameraNum++;
    }
}


void update() {

    float weightSum = 0;
    for (int i = 1; i < quantityOfSensors; i++) {
        float measurement;

        if (useFilter) {
            measurement = filters[i].updateValue((float) sensors[i]);
        }
        else {
            measurement = (float) sensors[i];
        }

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


//    sorriso
    sensors[1] = 100 * ((pointsFace[54].x - pointsFace[48].x) - 5.5) / (7.2 - 5.5);

    //sobrancelha esquerda
    sensors[5] = 100 * ((pointsFace[25].y - pointsFace[27].y) - 2.2) / (3.3 - 2.2);
    std::cout << "sombra esq" << (pointsFace[25].y - pointsFace[27].y) << std::endl;

    //sobrancelha direita
    sensors[6] = 100 * ((pointsFace[18].y - pointsFace[27].y) - 2.2) / (3.3 - 2.2);
    std::cout << "sombra dir" << (pointsFace[18].y - pointsFace[27].y) << std::endl;

    //olho esquerdo
    sensors[7] = 100 * ((points3d[37].y - points3d[41].y) - 0.15) / (0.9 - 0.15);
    std::cout << "olhO ESQ" << (pointsFace[37].y - pointsFace[41].y) << std::endl;

//    olho direito
    sensors[8] = 100 * ((pointsFace[37].y - pointsFace[41].y) - 0.15) / (0.9 - 0.15);
    std::cout << "olhO dir" << (pointsFace[37].y - pointsFace[41].y) << std::endl;
    //boca aberta
    sensors[9] = 100 * ((pointsFace[61].y - pointsFace[64].y) - 0.25) / (4.4 - 0.25);
    std::cout << "boca aberta" << (pointsFace[61].y - pointsFace[64].y) << std::endl;
    //boca fechada
//    sensors[10] = 100 * ((pointsFace[54].x - pointsFace[48].x) - 3.5) / (4.7 - 3.5);

//    sensors[10] = 100 - sensors[10];

//    sensors[1] = 100 - sensors[1];

    sensors[8] = 100 - sensors[8];
    if (sensors[8] > 80)
        sensors[8] = 100;
    if (sensors[8] > 70)
        sensors[8] = 80;
    if (sensors[8] < 60)
        sensors[8] = 0;

    sensors[7] = 100 - sensors[7];
    if (sensors[7] > 80)
        sensors[7] = 100;
    if (sensors[7] > 70)
        sensors[7] = 80;
    if (sensors[7] < 60)
        sensors[7] = 0;
//
//    std::cout << sensors[8] << std::endl;

//    cv::Mat olhoD;

//    std::cout << (int)points[39].x - (int)points[36].x << "  " << (int)(capHeight - points[41].y) - (int)(capHeight - points[37].y) << (int)pointsCam1[36].x <<
//            "  " << (int)pointsCam1[39].x << std::endl;
//    olhoD = grayScale(cv::Rect((int)points[36].x,(int)(capHeight - points[37].y),(int)points[39].x - (int)points[36].x +5,
//                               (int)(capHeight - points[41].y) - (int)(capHeight - points[37].y) +5));
//    if(!olhoD.empty())
//    cv::imshow("olhoD",olhoD);
//    cv::waitKey(50);
//
//    std::cout << cv::sum(olhoD)[0]/olhoD.total() << std:: endl;

//    if(cv::sum(olhoD)[0]/olhoD.total() < 123)
//        sensors[8] = 100;
//    else
//        sensors[8] = 0;


    //fix problems
    for (int i = 0; i < quantityOfSensors; i++) {
        if (sensors[i] > alpha_slider_max)
            sensors[i] = alpha_slider_max;
        if (sensors[i] < 0)
            sensors[i] = 0;
    }

}

