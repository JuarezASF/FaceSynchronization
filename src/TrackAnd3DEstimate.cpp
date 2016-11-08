#include <algorithm>
#include <iostream>
#include <fstream>

#include "tiny_obj_loader.h"

#include "global.h"
#include "util.h"


#include <mutex>

using namespace std;

std::vector<cv::Point3d> updateTracking(cv::Mat &A, cv::Mat &B);
void logMeasurements(vector<cv::Point3d> pointsFace);

//frame is used for web cam input; but the CSIRO face detection works only on gray scale images

//camera parameters
double f1 = (854.792781659906610 + 857.614193372593600) / 2;
double f2 = (834.378121568220080 + 835.838850269775660) / 2;
double b = 18;

std::vector<std::string> files = {
        "img/eyes-closed.jpg",
        "img/eyes-closed.jpg",
};

int main(int argc, char **argv) {

    if (!startFaceTracker()) {
        return 0;
    };
    if (!startCapture()) {
        return 0;
    };


    int i = 0;

    while (i + 1 < files.size()) {
        cv::Mat A, B;

        std::string fA = files[i];
        std::string fB = files[i + 1];
        i++;

        A = cv::imread(fA.c_str());
        B = cv::imread(fB.c_str());

        if (A.empty()) {
            std::cerr << "could not open file:" << fA << std::endl;
            return 0;
        }
        if (B.empty()) {
            std::cerr << "could not open file:" << fB << std::endl;
            return 0;
        }

        auto v3d = updateTracking(A, B);

        logMeasurements(v3d);


    }

    std::cout << "press to quit" << std::endl;
    char q = (char) (0xFF & cv::waitKey(0));


}

int cameraNum = 0;

std::vector<cv::Point_<double> > track(cv::Mat &frame) {
    std::vector<cv::Point_<double> > points;
    cv::Mat grayScale;

    if (frame.empty()) {
        std::cout << "Empty image received!" << std::endl;
        return points;
    }

    cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);

    //update state of tracker with new frame
    tracker->Reset();
    int detectionQuality = tracker->NewFrame(grayScale, params);

    if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
        std::cerr << "error while tracking!" << std::endl;
        return points;
    }

    //obtain points that were tracked
    points = tracker->getShape();

    for (auto p : points) {
//            putText(frame, std::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
        cv::circle(frame, p, 3, cv::Scalar(0, 0, 255), 3);
    }

    const std::string &winname = "Input#" + std::to_string(cameraNum);
    cvNamedWindow(winname.c_str(), CV_WINDOW_KEEPRATIO);

    int w = 250;
    cv::resizeWindow(winname, w, w);
    cv::moveWindow(winname.c_str(), w * (cameraNum % 2), w * (cameraNum / 2));
    imshow(winname, frame);

    cameraNum++;


    return points;

}

std::vector<cv::Point3d> updateTracking(cv::Mat &frameA, cv::Mat &frameB) {
    int detectionQuality;
    int cameraNum = 1;

    std::vector<cv::Point3d> points3d(66);

    std::vector<cv::Point_<double> > pointA = track(frameA);
    std::vector<cv::Point_<double> > pointB = track(frameB);

    if (pointA.size() != pointB.size())
        std::cerr << " points have differnt size!" << std::endl;

    int quantitOfPoints = (int) pointA.size();

    int hA = frameA.rows;
    int hB = frameB.rows;

    for (int i = 0; i < quantitOfPoints; i++) {
        pointA[i].y = hA - pointA[i].y;
        pointB[i].y = hB - pointB[i].y;

        points3d[i].z = f1 * f2 * b / (pointA[i].x * f2 - pointB[i].x * f1);
//                points3d[i].x = (pointsCam1[i].x / f1 + (points[i].x / f2 + b)) * points3d[i].z / 2;
        points3d[i].x = ((pointA[i].x / f1 + (pointB[i].x / f2)) * points3d[i].z + b)/ 2;
        points3d[i].y = (pointA[i].y / f1 + pointB[i].y / f2) * points3d[i].z / 2;
    }


    return points3d;
}


void logMeasurements(vector<cv::Point3d> pointsFace) {
    vector<double> x;
    //smile
    x.push_back(fabs(pointsFace[54].x - pointsFace[48].x));
    //left eyebrow
    x.push_back(fabs(pointsFace[25].y - pointsFace[27].y));
    //right eyebrow
    x.push_back(fabs(pointsFace[18].y - pointsFace[27].y));
    //left eye
    x.push_back(fabs(pointsFace[37].y - pointsFace[41].y));
    //right eye
    x.push_back(fabs(pointsFace[37].y - pointsFace[41].y));
    //open mouth
    x.push_back(fabs(pointsFace[61].y - pointsFace[64].y));

    vector<string> names = {
            "smile", "left eyebrow", "right eyebrow",
            "left eye", "right eye", "open mouth"
    };

    cout << "======================" << endl;
    cout << "======================" << endl;
    for( int i = 0 ; i < x.size(); i++){
        cout << names[i] << "\t" << x[i] << endl;
    }
    cout << "======================" << endl;
    cout << "======================" << endl;


}


