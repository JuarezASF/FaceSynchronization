#include <algorithm>
#include <iostream>

#include <GL/glew.h>


#include <GLFW/glfw3.h>

#include "tiny_obj_loader.h"

#include "trackball.h"
#include "global.h"
#include "display.h"

#include <opencv2/highgui.hpp>

#include <tracker/FaceTracker.hpp>


#include <sys/time.h>

int quantityOfPoses = 9;
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
};
std::vector<bool> invertNormals = {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
};

int quantityOfSensors = 2;

int *sliders = new int[quantityOfPoses];
float *weight = new float[quantityOfPoses];
double *sensors = new double[quantityOfSensors];

int alpha_slider_max = 100;

void update();

void getSensors(std::vector<cv::Point3d> pointsFace);

float t = 0.0f;


int main(int argc, char **argv) {

    ////////////////////////////////////////////FACETRACKER//////////////////////////////////////////////////////////

    FACETRACKER::FaceTracker *tracker = FACETRACKER::LoadFaceTracker();
    if (tracker == nullptr) {
        std::cerr << "Cannot load tracker!" << std::endl;
        return 0;
    }
    FACETRACKER::FaceTrackerParams *params = FACETRACKER::LoadFaceTrackerParams();

    if (params == nullptr) {
        std::cerr << "Cannot load tracker params!" << std::endl;
        return 0;

    }

//    cv::VideoCapture cam(0);
    std::vector<cv::VideoCapture> capture(2);
    capture.at(0).open("/home/rodrigo/ClionProjects/record_videos/out1.avi");
    capture.at(1).open("/home/rodrigo/ClionProjects/record_videos/out2.avi");
//    capture.at(0).open(1);
//    capture.at(1).open(2);

    if (!capture.at(0).isOpened() || !capture.at(1).isOpened()) {
        std::cerr << "cannot open camera!" << std::endl;
        return 0;
    }

    bool shouldFinish = false;

    //frame is used for web cam input; but the CSIRO face detection works only on gray scale images
    cv::Mat frame, grayScale;
    int detectionQuality;
    int cameraNum;
    std::vector<cv::Point_<double> > pointsCam1;

    double f1 = (854.792781659906610 + 857.614193372593600)/2;
    double f2 = (834.378121568220080 + 835.838850269775660) / 2;
    double b = 18;

    std::vector <cv::Point3d> points3d(66), points3dOld(66), points3dDiff(66);

//    std::cout << "press 'q' to quit" << std::endl;

////////////////////////////////////////////FACETRACKER//////////////////////////////////////////////////////////


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
    if (!LoadObjAndConvert(bmin, bmax, &drawObject, keyPoseFiles[0].c_str(), true, invertNormals[0])) {
        return -1;
    }
    for (auto poseFile : keyPoseFiles) {
        keyObjects.emplace_back();
        if (!LoadObjAndConvert(bmin, bmax, &keyObjects[keyObjects.size() - 1], poseFile.c_str(), false,
                               invertNormals[keyObjects.size() - 1])) {
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

        cameraNum = 1;

        for(auto cap : capture)
        {
            cap >> frame;

            cv::cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);

            //update state of tracker with new frame
            detectionQuality = tracker->NewFrame(grayScale, params);

            if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
                tracker->Reset();
            }

            std::cout << "Detection quality is: " << detectionQuality << std::endl;

            //obtain points that were tracked
            auto points = tracker->getShape();

            int count = 0;

            //draw point on input frame
            for (auto p : points) {
                cv::putText(frame, std::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
                count++;
//                cv::circle(frame, p, 1, cv::Scalar(255, 0, 0));

            }

            points3dOld = points3d;

            cv::imshow(std::to_string(cameraNum), frame);

            if(cameraNum == 1)
                pointsCam1 = points;
            else if(detectionQuality)
            {
                for(int i = 0; i < points.size(); i++)
                {
                    points[i].y = cap.get(CV_CAP_PROP_FRAME_HEIGHT) - points[i].y;
                    pointsCam1[i].y = cap.get(CV_CAP_PROP_FRAME_HEIGHT) - pointsCam1[i].y;

                    points3d[i].z = f1*f2*b/(pointsCam1[i].x*f2 - points[i].x*f1);
                    points3d[i].x = (pointsCam1[i].x/f1+points[i].x/f2)*points3d[i].z/2;
                    points3d[i].y = (pointsCam1[i].y/f1+points[i].y/f2)*points3d[i].z/2;
                }
            }

//            if(!points3dOld.empty())
//                for(int i = 0; i < points3d.size(); i++)
//                {
//                    points3dDiff[i] = points3d[i] - points3dOld[i];
//                }

            std::cout << points3d[61].y - points3d[64].y << std::endl;
            // Z = f1*f2*b/(x1*f2 - x2*f1);

            cameraNum++;

            if (char q = (char) (cv::waitKey(30) & 0xFF) == 'q') {
                shouldFinish = true;
            }
        }

        getSensors(points3d);

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
    for (int i = 0; i < quantityOfSensors; i++) {
        weight[i] = ((float) sensors[i]) / alpha_slider_max;
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

void getSensors(std::vector<cv::Point3d> pointsFace){

    sensors[0] = 0;
    //100*(diffPoints-dmin)/(dmax-dmin)
    sensors[1] = 100*((pointsFace[61].y - pointsFace[64].y)-0.2)/(3.5-0.2);

    if(sensors[1] > 100)
        sensors[1] = 100;
    if(sensors[1] < 0)
        sensors [1] = 0;

//    sensors[0] = 100*((pointsFace[61].y - pointsFace[64].y)-0.2)/(3.5-0.2);
//
//    if(sensors[0] > 100)
//        sensors[0] = 100;
//    if(sensors[0] < 0)
//        sensors [0] = 0;


}
