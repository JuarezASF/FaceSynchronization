#include <algorithm>
#include <iostream>
#include <fstream>

#include <GL/glew.h>


#include <GLFW/glfw3.h>

#include "tiny_obj_loader.h"

#include "trackball.h"
#include "global.h"
#include "display.h"
#include "util.h"
#include "Filter.h"
#include <map>

#include <opencv2/highgui.hpp>

#include <tracker/FaceTracker.hpp>


#include <sys/time.h>
#include <mutex>

#define HIGHRES 0

#if HIGHRES > 0
#define QUANTITY_POSES  8
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
#define QUANTITY_POSES 12
#define QUANTITY_SENSORS QUANTITY_POSES

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


int *sliders = new int[QUANTITY_POSES];
float *weight = new float[QUANTITY_POSES];
double *sensors = new double[QUANTITY_SENSORS];


bool useFilter = true;
WindowFilter *filters;

int alpha_slider_max = 100;

void update();

void updateSensors(std::vector<cv::Point3d> pointsFace);

bool updateTracking();

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


int quantityOfFilterConfigurations;
int *sizeOfFilter;
double **filterConfigurations;
int *filterUsedForSensor;
int *indices;


bool mustUpdateFilters = false;

// to protect modifications on sensor configuration
std::mutex sensorsConfigurationLock;

void updateFilter(int configuration, void *userData);

//to produce sensor/filter graph
std::vector<float> sensorOutput;
std::vector<float> filterOutput;

int SENSOR_TO_TRACK = 5;

std::map<int, int> sensorToDefaultFilterConfig = {
        {7, 3},
        {8, 3}
};

std::map<int, std::string> sensorToNameMap = {
        {0, "neutro"},
        {1, "feliz"},
        {2, "bravo"},
        {3, "left cheek"},
        {4, "right cheek"},
        {5, "left eyebrow"},
        {6, "right eyebrow"},
        {7, "left eye"},
        {8, "right eye"},
        {9, "open mouth"},
        {10, "close mouth"},
        {11, "nose"},
};

int *d_min = new int[QUANTITY_SENSORS]{
        0,
        59,
        0,
        0,
        0,
        24,
        24,
        2,
        2,
        3,
        0,
        0
};

int *d_max = new int[QUANTITY_SENSORS]{
        0,
        79,
        0,
        0,
        0,
        37,
        36,
        200,
        200,
        43,
        0,
        0

};

int main(int argc, char **argv) {

    if (!startFaceTracker()) {
        return 0;
    };
    if (!startCapture()) {
        return 0;
    };

    //zero out weights
    for (int k = 0; k < QUANTITY_POSES; k++) {
        weight[k] = 0.0;
        sliders[k] = (int) 0;
    }
    //initialize open cv windows for slide trackers
    cvNamedWindow("Shapes", CV_WINDOW_NORMAL);
    cv::moveWindow("Shapes", 0, 0);

    cvNamedWindow("FilterControl", CV_WINDOW_KEEPRATIO);
    cv::moveWindow("FilterControl", 100, 0);

    cvNamedWindow("GainWindow", CV_WINDOW_KEEPRATIO);
    cv::moveWindow("GainWindow", 200, 0);

    cvNamedWindow("OffsetWindow", CV_WINDOW_KEEPRATIO);
    cv::moveWindow("OffsetWindow", 300, 0);

    cvNamedWindow("Input#1", CV_WINDOW_KEEPRATIO);
    cv::moveWindow("Input#1", 400, 400);


    //initialize filters

    quantityOfFilterConfigurations = 13;
    filterConfigurations = new double *[quantityOfFilterConfigurations];
    sizeOfFilter = new int[quantityOfFilterConfigurations];

    filterConfigurations[0] = new double[1]{1.0};
    sizeOfFilter[0] = 1;

    filterConfigurations[1] = new double[3]{0.25, 0.5, 0.25};
    sizeOfFilter[1] = 3;

    filterConfigurations[2] = new double[4]{0.64f, 0.16f, 0.16f};
    sizeOfFilter[2] = 3;

    // wc=0.1 pi
    filterConfigurations[3] = new double[17]{
            0.002539993835013,
            0.005744201059608,
            0.014708336514843,
            0.031456060871751,
            0.055482250809604,
            0.083441910965449,
            0.109888911438294,
            0.128859581669095,
            0.135757505672686,
            0.128859581669095,
            0.109888911438294,
            0.083441910965449,
            0.055482250809604,
            0.031456060871751,
            0.014708336514843,
            0.005744201059608,
            0.002539993835013
    };
    sizeOfFilter[3] = 17;

// wc =     0.188888888888889 pi
    filterConfigurations[4] = new double[17]{
            -0.003247205518653,
            -0.004527442172625,
            -0.004729671439724,
            0.004107097443012,
            0.030470491470910,
            0.075856260831913,
            0.130335129175133,
            0.175330085491631,
            0.192810509436804,
            0.175330085491631,
            0.130335129175133,
            0.075856260831913,
            0.030470491470910,
            0.004107097443012,
            -0.004729671439724,
            -0.004527442172625,
            -0.003247205518653
    };
    sizeOfFilter[4] = 17;

//            0.277777777777778

    filterConfigurations[5] = new double[17]{
            0.002035478571551,
            -0.000903498173481,
            -0.009814607235028,
            -0.021660835977413,
            -0.014621249602320,
            0.037790415080522,
            0.134918562616124,
            0.234084889513956,
            0.276341690412178,
            0.234084889513956,
            0.134918562616124,
            0.037790415080522,
            -0.014621249602320,
            -0.021660835977413,
            -0.009814607235028,
            -0.000903498173481,
            0.002035478571551};
    sizeOfFilter[5] = 17;


//            0.366666666666667 pi

    filterConfigurations[6] = new double[17]{
            0.000660540283494,
            0.005106023428239,
            0.006683167383032,
            -0.011563272541624,
            -0.042654859491909,
            -0.023432324993101,
            0.102144518401223,
            0.280072802152267,
            0.365966810756758,
            0.280072802152267,
            0.102144518401223,
            -0.023432324993101,
            -0.042654859491909,
            -0.011563272541624,
            0.006683167383032,
            0.005106023428239,
            0.000660540283494};
    sizeOfFilter[6] = 17;

//            0.455555555555556 pi

    filterConfigurations[7] = new double[17]{
            -0.002871493040968,
            -0.002935399685742,
            0.008496974094026,
            0.017815249422312,
            -0.022855517410369,
            -0.069661105698092,
            0.038098411211602,
            0.305295731606352,
            0.457234299001755,
            0.305295731606352,
            0.038098411211602,
            -0.069661105698092,
            -0.022855517410369,
            0.017815249422312,
            0.008496974094026,
            -0.002935399685742,
            -0.002871493040968};
    sizeOfFilter[7] = 17;

//            0.544444444444444 pi

    filterConfigurations[8] = new double[17]{
            0.002855096367657,
            -0.002918638095518,
            -0.008448455046144,
            0.017713521568349,
            0.022725008839747,
            -0.069263329915145,
            -0.037880863338985,
            0.303552445342762,
            0.543330428554551,
            0.303552445342762,
            -0.037880863338985,
            -0.069263329915145,
            0.022725008839747,
            0.017713521568349,
            -0.008448455046144,
            -0.002918638095518,
            0.002855096367657};
    sizeOfFilter[8] = 17;

//            0.633333333333333 pi

    filterConfigurations[9] = new double[17]{
            -0.000661317851602,
            0.005112034085085,
            -0.006691034606194,
            -0.011576884477457,
            0.042705071506567,
            -0.023459908819657,
            -0.102264759848886,
            0.280402495411475,
            0.632868609201339,
            0.280402495411475,
            -0.102264759848886,
            -0.023459908819657,
            0.042705071506567,
            -0.011576884477457,
            -0.006691034606194,
            0.005112034085085,
            -0.000661317851602};
    sizeOfFilter[9] = 17;

//            0.722222222222222

    filterConfigurations[10] = new double[17]{
            -0.002051740735801,
            -0.000910716542616,
            0.009893019632551,
            -0.021833892121247,
            0.014738063979977,
            0.038092336184331,
            -0.135996475130784,
            0.235955077181294,
            0.724228655104588,
            0.235955077181294,
            -0.135996475130784,
            0.038092336184331,
            0.014738063979977,
            -0.021833892121247,
            0.009893019632551,
            -0.000910716542616,
            -0.002051740735801};
    sizeOfFilter[10] = 17;

//            0.811111111111111 pi

    filterConfigurations[11] = new double[17]{
            0.003171639535508,
            -0.004422083698411,
            0.004619606871823,
            0.004011520845113,
            -0.029761410191120,
            0.074091003630133,
            -0.127302089807016,
            0.171249964843599,
            0.808683695940741,
            0.171249964843599,
            -0.127302089807016,
            0.074091003630133,
            -0.029761410191120,
            0.004011520845113,
            0.004619606871823,
            -0.004422083698411,
            0.003171639535508};
    sizeOfFilter[11] = 17;

//            0.900000000000000 pi

    filterConfigurations[12] = new double[17]{
            -0.001873729287127,
            0.004237442472562,
            -0.010850199915004,
            0.023204836838796,
            -0.040928728575873,
            0.061554303870561,
            -0.081064004508224,
            0.095058487454551,
            0.901323183299516,
            0.095058487454551,
            -0.081064004508224,
            0.061554303870561,
            -0.040928728575873,
            0.023204836838796,
            -0.010850199915004,
            0.004237442472562,
            -0.001873729287127};
    sizeOfFilter[12] = 17;


    filters = new WindowFilter[QUANTITY_SENSORS];
    filterUsedForSensor = new int[QUANTITY_SENSORS];
    indices = new int[QUANTITY_SENSORS];

    for (int k = 0; k < QUANTITY_SENSORS; k++) {
        if (sensorToDefaultFilterConfig.find(k) != sensorToDefaultFilterConfig.end()) {
            const int defaultValue = sensorToDefaultFilterConfig[k];
            filterUsedForSensor[k] = defaultValue;
        } else {
            filterUsedForSensor[k] = 4;

        }

        indices[k] = k;
        filters[k] = WindowFilter(sizeOfFilter[filterUsedForSensor[k]], filterConfigurations[filterUsedForSensor[k]]);
        std::string trackName = "sensor on " + sensorToNameMap[k];

        cv::createTrackbar(trackName.c_str(), "FilterControl", &(filterUsedForSensor[k]),
                           quantityOfFilterConfigurations - 1, updateFilter, indices + k);

        trackName = "offset on " + sensorToNameMap[k];
        cv::createTrackbar(trackName.c_str(), "OffsetWindow", &d_min[k],
                           100, nullptr, nullptr);

        trackName = "gain on " + sensorToNameMap[k];
        cv::createTrackbar(trackName.c_str(), "GainWindow", &d_max[k],
                           200, nullptr, nullptr);

    }


    //initialize sliders
    int sliderIdx = 0;
    for (auto poseName : keyPoseNames) {
        std::string windowName = "Shapes";
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
    glfwSetWindowPos(window, 400, 0);
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

    /////////////////////////////////

    while (glfwWindowShouldClose(window) == GL_FALSE) {
        if (!updateTracking()) {
            std::cout << "Finishing execution due to end of video" << std::endl;
            break;
        }

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

    std::string dataFileName = "data.txt";
    std::cout << "writing sensor data to:" << dataFileName << std::endl;

    std::ofstream f;
    f.open(dataFileName);

    unsigned long quantityOfSamples = filterOutput.size();
    for (int i = 0; i < quantityOfSamples; i++) {
        f << i << "\t" << sensorOutput[i] << "\t" << filterOutput[i] << std::endl;
    }
    f.close();

}

bool updateTracking() {
    int detectionQuality;
    int cameraNum = 1;

    for (auto cap : capture) {

        cap >> frame;
        if (frame.empty()) {
            std::cout << "end of video reached!" << std::endl;
            return false;
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
            // uncomment if you want to print tracker number
//            putText(frame, std::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
            count++;
            cv::circle(frame, p, 1, cv::Scalar(255, 0, 0));

        }

        points3dOld = points3d;


        if (cameraNum == 1) {
            pointsCam1 = points;
            imshow("Input#" + std::to_string(cameraNum), frame);

        }
        else if (detectionQuality) {
            capHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
            for (int i = 0; i < points.size(); i++) {
                points[i].y = cap.get(CV_CAP_PROP_FRAME_HEIGHT) - points[i].y;
                pointsCam1[i].y = cap.get(CV_CAP_PROP_FRAME_HEIGHT) - pointsCam1[i].y;

                points3d[i].z = f1 * f2 * b / (pointsCam1[i].x * f2 - points[i].x * f1);
                points3d[i].x = ((pointsCam1[i].x / f1 + (points[i].x / f2)) * points3d[i].z + b) / 2;
                points3d[i].y = (pointsCam1[i].y / f1 + points[i].y / f2) * points3d[i].z / 2;

//                std::cout << "Y1:  " << (pointsCam1[i].y / f1) * points3d[i].z  << "\tY2: " << (points[i].y / f2) * points3d[i].z << std::endl;
            }
            imshow("Input#" + std::to_string(cameraNum), frame);

        }



        cameraNum++;
    }

    return true;
}


void update() {

    float weightSum = 0;
    std::lock_guard<std::mutex> guard(sensorsConfigurationLock);
    //skip sensort 0 ( for neutral pose)
    for (int i = 1; i < QUANTITY_SENSORS; i++) {
        float measurement;

        float sensorValue = (float) sensors[i];
        if (useFilter) {
            measurement = filters[i].updateValue(sensorValue);
        }
        else {
            measurement = sensorValue;
        }

        if (i == SENSOR_TO_TRACK) {
            sensorOutput.push_back(sensorValue);
            filterOutput.push_back(measurement);
        }

        weight[i] = measurement / 100.0f;
        weightSum += weight[i];
    }
    guard.~lock_guard();

    weight[0] = 1.0f - weightSum;


    for (int i = 0; i < drawObject.size(); i++) {
        for (int j = 0; j < drawObject[i].attributes.size(); j++) {
            drawObject[i].attributes[j] = 0.0;
            for (int k = 0; k < QUANTITY_SENSORS; k++) {
                drawObject[i].attributes[j] += weight[k] * keyObjects[k][i].attributes[j];
            }
        }
    }
}

int quantityOfUsedSensors = 6;
int usedSensors[6] = {1, 5, 6, 7, 8, 9};

void updateSensors(std::vector<cv::Point3d> pointsFace) {
    //default behavior for sensors is to follow tracker
    for (int i = 0; i < QUANTITY_POSES; i++)
        sensors[i] = sliders[i];

    //smile
    sensors[1] = fabs(pointsFace[54].x - pointsFace[48].x);
    //left eyebrow
    sensors[5] = pointsFace[24].y - pointsFace[42].y;
    //right eyebrow
    sensors[6] = pointsFace[19].y - pointsFace[39].y;
    //left eye
    sensors[7] = points3d[43].y - points3d[47].y;
    //right eye
    sensors[8] = fabs(pointsFace[37].y - pointsFace[41].y);
    //open mouth
    sensors[9] = fabs(pointsFace[61].y - pointsFace[64].y);

//    std::cout << "smile: " << sensors[1] << std::endl;
    std::cout << "left eyebrow: " << sensors[5] << std::endl;
//    std::cout << "right eyebrow: " << sensors[6] << std::endl;
//    std::cout << "left eye: " << sensors[7] << std::endl;
//    std::cout << "right eye: " << sensors[8] << std::endl;
//    std::cout << "open mouth: " << sensors[9] << std::endl;

    for(int k = 0; k < quantityOfUsedSensors; k++){
        int i = usedSensors[k];
        sensors[i] = 100.0 * (sensors[i] - d_min[i]/10.0) *  1/((d_max[i] - d_min[i])/10.0);
//        std::cout << i << " s: " << sensors[i] << std:: endl;
    }

    //trick to improve left eye
    sensors[7] = 100 - sensors[7];
    if (sensors[7] > 80)
        sensors[7] = 100;
    if (sensors[7] > 70)
        sensors[7] = 80;
    if (sensors[7] < 60)
        sensors[7] = 0;
    //trick to improve right eye
    sensors[8] = 100 - sensors[8];
    if (sensors[8] > 80)
        sensors[8] = 100;
    if (sensors[8] > 70)
        sensors[8] = 80;
    if (sensors[8] < 60)
        sensors[8] = 0;

//    std::cout << pointsFace[24].y - pointsFace[42].y << " 7 " << sensors[7] << " 8 " << sensors[8] << std:: endl;


    // truncates at 100 and 0 all sensors
    for(int k = 0; k < quantityOfUsedSensors; k++){
        int i = usedSensors[k];
        if (sensors[i] > alpha_slider_max)
            sensors[i] = alpha_slider_max;
        if (sensors[i] < 0)
            sensors[i] = 0;
    }

}

void updateFilter(int configuration, void *userData) {
    //make sure we don't modify filters while they are being read/updated
    std::lock_guard<std::mutex> guard(sensorsConfigurationLock);

    int filterToUpdate = *((int *) (userData));

    std::cout << "Changing filter on sensor: " << filterToUpdate << " to configuration:" << configuration << "...";


    filterUsedForSensor[filterToUpdate] = configuration;

    filters[filterToUpdate].updateCoefficients(sizeOfFilter[configuration], filterConfigurations[configuration]);

    std::cout << "[DONE]" << std::endl;

}

