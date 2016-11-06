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

#define quantityOfSensorts  12

double *sensors = new double[quantityOfSensorts]{0,0,0,0,0,0,0,0,0,0,0,0};

void updateSensors(std::vector<cv::Point_<double> > pointsFace) {
    //smile
    sensors[1] = fabs(pointsFace[54].x - pointsFace[48].x);
    //left eyebrow
    sensors[5] = fabs(pointsFace[25].y - pointsFace[27].y);
    //right eyebrow
    sensors[6] = fabs(pointsFace[18].y - pointsFace[27].y);
    //left eye
    sensors[7] = fabs(pointsFace[37].y - pointsFace[41].y);
    //right eye
    sensors[8] = fabs(pointsFace[37].y - pointsFace[41].y);
    //open mouth
    sensors[9] = fabs(pointsFace[61].y - pointsFace[64].y);

    for (int i = 0; i < quantityOfSensorts ; i++){
        cout << "\t sensor#" << i << ":" << sensors[i] << endl;
    }


}

vector<string> callibFiles = {
        "input.png",
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

    for (string fileName : callibFiles) {
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

        cout << fileName << endl;
        updateSensors(points);

        int count = 0;

        //draw point on input frame
        for (auto p : points) {
//            putText(img, std::__cxx11::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
            circle(img, p, 8, cv::Scalar(0, 0, 255), 3);
            count++;
        }

        imgs.push_back(cv::Mat(img));


    }
    int k = 0;
    for (int i = 0; i < callibFiles.size(); i++) {
        string windowName = callibFiles[i];
        cv::namedWindow(windowName.c_str(), CV_WINDOW_KEEPRATIO);
        cv::moveWindow(windowName.c_str(), (k%3) * 240, (k / 3)*240 );
        cv::imshow(windowName.c_str(), imgs[i]);
        k++;

    }

    cv::waitKey(0);

    return 0;


}

