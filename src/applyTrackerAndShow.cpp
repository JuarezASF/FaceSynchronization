//
// Created by juarez on 22/09/16.
//
#include <iostream>
#include <vector>
#include <string>
#include <string>
#include <opencv2/highgui.hpp>

#include <tracker/FaceTracker.hpp>

using namespace std;

void usage() {
    cerr << "program input img" << endl;
}

int quantityOfSensorts = 2;
double *sensors = new double[quantityOfSensorts];

void updateSensors(std::vector<cv::Point3d> pointsFace) {
    //default behavior for sensors is to follow tracker

    sensors[1] = 100 * ((pointsFace[61].y - pointsFace[64].y) - 0.2) / (3.5 - 0.2);


}

vector<string> callibFiles = {
        "img/mouth-opened.jpg",
        "img/mouth-duck.jpg",
        "img/mouth-smile.jpg",
        "img/eyes-opened.jpg",
        "img/eyes-closed.jpg"
};


int main(int argc, char **argv) {
    vector<cv::Mat> imgs;
    FACETRACKER::FaceTracker *tracker;
    FACETRACKER::FaceTrackerParams *params;

    tracker = FACETRACKER::LoadFaceTracker();
    params = FACETRACKER::LoadFaceTrackerParams();

    for(string fileName : callibFiles){
        cv::Mat img, gray;
        img = cv::imread(fileName.c_str(), CV_LOAD_IMAGE_COLOR);

        if (!img.data) {
            cerr << "could not open:" << fileName << endl;
            return 0;
        }
        cvtColor(img, gray, cv::COLOR_BGR2GRAY);


        if (tracker == nullptr) {
            std::cerr << "Cannot load tracker!" << std::endl;
            return 0;
        }

        if (params == nullptr) {
            std::cerr << "Cannot load tracker params!" << std::endl;
            return 0;

        }

        //update state of tracker with new frame
        int detectionQuality = tracker->NewFrame(gray, params);

        if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
            tracker->Reset();
        }

        //obtain points that were tracked
        auto points = tracker->getShape();

        int count = 0;

        //draw point on input frame
        for (auto p : points) {
            putText(img, std::__cxx11::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
            count++;
        }

        imgs.push_back(cv::Mat(img));


    }
    for(int i = 0; i < callibFiles.size(); i++){
        string windowName = callibFiles[i];
        cv::namedWindow(windowName.c_str(), CV_WINDOW_KEEPRATIO);
        cv::imshow(windowName.c_str(), imgs[i]);

    }

    cv::waitKey(0);

    return 0;


}

