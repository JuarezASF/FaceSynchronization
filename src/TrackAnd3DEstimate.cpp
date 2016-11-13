#include <algorithm>
#include <iostream>
#include <fstream>

#include "tiny_obj_loader.h"

#include "global.h"
#include "util.h"


#include <mutex>


using namespace std;

typedef std::tuple<vector<cv::Point3d>, vector<cv::Point2d>, vector<cv::Point2d>> ReturnedPoints;

ReturnedPoints updateTracking(cv::Mat &A, cv::Mat &B);

void logMeasurements(ReturnedPoints pointsFace);

//frame is used for web cam input; but the CSIRO face detection works only on gray scale images

//camera parameters
double f1 = (854.792781659906610 + 857.614193372593600) / 2;
double f2 = (834.378121568220080 + 835.838850269775660) / 2;
double b = 18;

namespace fs = boost::filesystem;

std::fstream f;

void runOnDirectory(std::string dirName);

int main(int argc, char **argv) {

    std::string baseName = "exp/exp13112016/exp";
    int quantityOfExp =  10;
    int distances[] = {35,40,45,50,55,60,65,70,75,80};

    for(int i = 0; i < quantityOfExp; i++){

        string name = baseName + to_string(distances[i]) + "cmNeutro";
        runOnDirectory(name);
        string nameB = baseName + to_string(distances[i]) + "cmSorrindo";
        runOnDirectory(nameB);

    }



}

int cameraNum = 0;

void runOnDirectory(std::string dirName) {
    if (!startFaceTracker()) {
        return;
    };
    if (!startCapture()) {
        return;
    };

    std::vector<std::pair<std::string, std::string>> imagePair =
            getFilesOnDir(dirName);

    std::string dataFileName(dirName + "/data.txt");

    cout << "logging output to:" << dataFileName << endl;

    f.open(dataFileName.c_str(), ios_base::out);


    for (auto pair : imagePair) {
        cv::Mat A, B;

        std::string fA = pair.first;
        std::string fB = pair.second;

        cout << fA << endl;
        cout << fB << endl;

        A = cv::imread(fA.c_str());
        B = cv::imread(fB.c_str());

        if (A.empty()) {
            std::cerr << "could not open file:" << fA << std::endl;
            return;
        }
        if (B.empty()) {
            std::cerr << "could not open file:" << fB << std::endl;
            return;
        }

        auto v3d = updateTracking(A, B);

        logMeasurements(v3d);


    }

    f.flush();
    f.close();

}

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
        tracker->Reset();
        detectionQuality = tracker->NewFrame(grayScale, params);

        if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
            std::cerr << "error while tracking!" << std::endl;
            return points;
        }
    }

    //obtain points that were tracked
    points = tracker->getShape();

    for (auto p : points) {
//            putText(frame, std::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
        cv::circle(frame, p, 3, cv::Scalar(0, 0, 255), 1);
    }

//    const std::string &winname = "Input#" + std::to_string(cameraNum);
//    cvNamedWindow(winname.c_str(), CV_WINDOW_FREERATIO);
//    int w = 100;
//    cv::resizeWindow(winname, w, w);
//    cv::moveWindow(winname.c_str(), w * (cameraNum % 6), w * (cameraNum / 6));
//    imshow(winname, frame);

    cameraNum++;


    return points;

}


ReturnedPoints updateTracking(cv::Mat &frameA, cv::Mat &frameB) {
    int detectionQuality;
    int cameraNum = 1;

    std::vector<cv::Point3d> points3d(66);
    std::vector<cv::Point2d> points2dLeft;
    std::vector<cv::Point2d> points2dRight;

    std::vector<cv::Point2d> pointA = track(frameA);
    std::vector<cv::Point2d> pointB = track(frameB);

    if (pointA.size() != pointB.size()) {
        std::cerr << " points have differnt size!" << std::endl;
        std::cerr << " left has " << pointA.size() << " and right has:" << pointB.size() << endl;
        return make_tuple(points3d, points2dLeft, points2dRight);

    }

    int quantitOfPoints = (int) pointA.size();

    int hA = frameA.rows;
    int hB = frameB.rows;

    for (int i = 0; i < quantitOfPoints; i++) {
        pointA[i].y = hA - pointA[i].y;
        pointB[i].y = hB - pointB[i].y;

        points2dLeft.emplace_back(pointA[i].x, pointA[i].y);
        points2dRight.emplace_back(pointB[i].x, pointB[i].y);

        points3d[i].z = f1 * f2 * b / (pointA[i].x * f2 - pointB[i].x * f1);
        points3d[i].x = ((pointA[i].x / f1 + (pointB[i].x / f2)) * points3d[i].z + b) / 2.0;
        points3d[i].y = (pointA[i].y / f1 + pointB[i].y / f2) * points3d[i].z / 2.0;
    }


    return make_tuple(points3d, points2dLeft, points2dRight);
}

void logMeasurements(ReturnedPoints data) {

    auto pointsFace = std::get<0>(data);
    auto leftRawPoint = std::get<1>(data);
    auto rightRawPoint = std::get<2>(data);

    if (pointsFace.empty() || leftRawPoint.empty() || rightRawPoint.empty()) {
        cout << "skiping data due to empyt points" << endl;
        return;
    }

    double corrected3d = fabs(pointsFace[54].x - pointsFace[48].x);
    double rawLeft = fabs(leftRawPoint[54].x - leftRawPoint[48].x);
    double rawRight = (fabs(rightRawPoint[54].x - rightRawPoint[48].x));


    f << corrected3d << "\t" << rawLeft << "\t" << rawRight << endl;

}


