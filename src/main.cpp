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
#include "FIR.h"


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


/**
 * input rom trackbar
 */
int *sliders = new int[QUANTITY_POSES];
/**
 * misturing weights
 */
float *weight = new float[QUANTITY_POSES];
/**
 * for the ratio of distances
 */
double *sensors = new double[QUANTITY_SENSORS];


bool useFilter = true;

Fir1 **filters;

/**
 * defines the maximum input from trackbars
 */
int alpha_slider_max = 100;

void update();

void updateSensors(std::vector<cv::Point3d> pointsFace);

bool updateTracking();

void updateFilter(int configuration, void *userData);

float t = 0.0f;

//frame is used for web cam input; but the CSIRO face detection works only on gray scale images
cv::Mat frame, grayScale;
std::vector<cv::Point3d> points3d(66);
std::vector<cv::Point_<double> > pointsCam1, points;

//camera parameters
double f1 = (854.792781659906610 + 857.614193372593600) / 2;
double f2 = (834.378121568220080 + 835.838850269775660) / 2;
double b = 18;


int quantityOfFilterConfigurations;
/**
 * indicates the quantity of coefficients for the i-th filter
 */
int *sizeOfFilter;
/**
 * at position i contains the array of coefficients for the i-th filter
 */
double **filterConfigurations;

std::vector<std::string> filterDescription;

/**
 * indicates which configuration is used for which filter
 */
int *filterUsedForSensor;
/**
 * used so that trackbar callback know which filter to set
 */
int *indices;


bool mustUpdateFilters = false;

/**
 * used to protect modifications on sensor configuration
 */
std::mutex sensorsConfigurationLock;


//to produce sensor/filter graph
std::vector<float> sensorOutput;
std::vector<float> *filterOutput;

/**
 * vectors above are filled with information for this sensor
 */
int SENSOR_TO_TRACK = 1;

int DEFAULT_FILTER_CONFIG = 1;

Fir1 **filterForTrackedSensor;

/**
 * sensor index -> index of filter to use as default
 */
std::map<int, int> sensorToDefaultFilterConfig = {
        {7, 3},
        {8, 3}
};


/**
 * used to nmae trackbars
 */
std::map<int, std::string> sensorToNameMap = {
        {0,  "neutro"},
        {1,  "feliz"},
        {2,  "bravo"},
        {3,  "left cheek"},
        {4,  "right cheek"},
        {5,  "left eyebrow"},
        {6,  "right eyebrow"},
        {7,  "left eye"},
        {8,  "right eye"},
        {9,  "open mouth"},
        {10, "close mouth"},
        {11, "nose"},
};

/**
 * calibration to environment
 * TODO: read this from file
 */
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

/**
 * same as above
 * TODO: read this from file
 */
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
    filterForTrackedSensor = new Fir1 *[quantityOfFilterConfigurations];
    filterOutput = new std::vector<float>[quantityOfFilterConfigurations];

    setFilterConfigurations(filterConfigurations, sizeOfFilter, filterDescription);

    for (int k = 0; k < quantityOfFilterConfigurations; k++) {
        filterForTrackedSensor[k] = new Fir1(filterConfigurations[k], (unsigned int) sizeOfFilter[k]);
    }

    filters = new Fir1 *[QUANTITY_SENSORS];
    filterUsedForSensor = new int[QUANTITY_SENSORS];
    indices = new int[QUANTITY_SENSORS];


    for (int k = 0; k < QUANTITY_SENSORS; k++) {
        if (sensorToDefaultFilterConfig.find(k) != sensorToDefaultFilterConfig.end()) {
            const int defaultValue = sensorToDefaultFilterConfig[k];
            filterUsedForSensor[k] = defaultValue;
        } else {
            filterUsedForSensor[k] = DEFAULT_FILTER_CONFIG;

        }

        indices[k] = k;
        filters[k] = new Fir1(filterConfigurations[filterUsedForSensor[k]], sizeOfFilter[filterUsedForSensor[k]]);
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

    std::string dataFileName = "data_s" + std::to_string(SENSOR_TO_TRACK) + ".txt";

    std::cout << "writing sensor data to:" << dataFileName << std::endl;
    std::cout << "data recorded for sensor:" << sensorToNameMap[SENSOR_TO_TRACK] << std::endl;

    std::ofstream f;
    f.open(dataFileName);

    unsigned long quantityOfSamples = filterOutput[0].size();
    for (int i = 0; i < quantityOfSamples; i++) {
        f << i << "\t";
        for (int k = 0; k < quantityOfFilterConfigurations; k++) {
            f << filterOutput[k][i] << "\t";
        }
        f << std::endl;
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
//            putText(frame, std::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
            count++;
            cv::circle(frame, p, 1, cv::Scalar(255, 0, 0));

        }


        if (cameraNum == 1) {
            pointsCam1 = points;
            imshow("Input#" + std::to_string(cameraNum), frame);

        }
        else if (detectionQuality) {
            int capHeight = (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT);
            for (int i = 0; i < points.size(); i++) {
                points[i].y = capHeight - points[i].y;
                pointsCam1[i].y = capHeight - pointsCam1[i].y;

                points3d[i].z = f1 * f2 * b / (pointsCam1[i].x * f2 - points[i].x * f1);
                points3d[i].x = ((pointsCam1[i].x / f1 + (points[i].x / f2)) * points3d[i].z + b) / 2;
                points3d[i].y = (pointsCam1[i].y / f1 + points[i].y / f2) * points3d[i].z / 2;

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
            measurement = (float) filters[i]->updateValue(sensorValue);
        }
        else {
            measurement = sensorValue;
        }

        if (i == SENSOR_TO_TRACK) {
            for (int k = 0; k < quantityOfFilterConfigurations; k++) {
                float filterOutputValue = (float) filterForTrackedSensor[k]->updateValue(sensorValue);
                filterOutput[k].push_back(filterOutputValue);
            }
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
//    std::cout << "left eyebrow: " << sensors[5] << std::endl;
//    std::cout << "right eyebrow: " << sensors[6] << std::endl;
//    std::cout << "left eye: " << sensors[7] << std::endl;
//    std::cout << "right eye: " << sensors[8] << std::endl;
//    std::cout << "open mouth: " << sensors[9] << std::endl;

    for (int k = 0; k < quantityOfUsedSensors; k++) {
        int i = usedSensors[k];
        sensors[i] = 100.0 * (sensors[i] - d_min[i] / 10.0) / ((d_max[i] - d_min[i]) / 10.0);
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
    for (int k = 0; k < quantityOfUsedSensors; k++) {
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

    std::cout << "Changing filter on sensor: " << filterToUpdate << " to configuration:" << configuration <<
    ":" << filterDescription[configuration] << "...";

    filterUsedForSensor[filterToUpdate] = configuration;

    filters[filterToUpdate]->updateCoefficients(filterConfigurations[configuration], sizeOfFilter[configuration]);

    std::cout << "[DONE]" << std::endl;

}

