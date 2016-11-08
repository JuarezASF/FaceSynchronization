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

//std::vector<std::string> files = {
//        "img/eyes-closed.jpg",
//        "img/eyes-closed.jpg",
//};


//std::vector<std::string> files = {
//        "img/varyingDistanceExpStart30step5end60/image01.png",
//        "img/varyingDistanceExpStart30step5end60/image02.png",
//        "img/varyingDistanceExpStart30step5end60/image03.png",
//        "img/varyingDistanceExpStart30step5end60/image04.png",
//        "img/varyingDistanceExpStart30step5end60/image05.png",
//        "img/varyingDistanceExpStart30step5end60/image06.png",
//        "img/varyingDistanceExpStart30step5end60/image07.png",
//        "img/varyingDistanceExpStart30step5end60/image08.png",
//        "img/varyingDistanceExpStart30step5end60/image09.png",
//        "img/varyingDistanceExpStart30step5end60/image10.png",
//        "img/varyingDistanceExpStart30step5end60/image11.png",
//        "img/varyingDistanceExpStart30step5end60/image12.png",
//};

//std::vector<std::string> files = {
//        "img/sameDistanceExp35cm/image01.png",
//        "img/sameDistanceExp35cm/image02.png",
//        "img/sameDistanceExp35cm/image03.png",
//        "img/sameDistanceExp35cm/image04.png",
//        "img/sameDistanceExp35cm/image05.png",
//        "img/sameDistanceExp35cm/image06.png",
//        "img/sameDistanceExp35cm/image07.png",
//        "img/sameDistanceExp35cm/image08.png",
//        "img/sameDistanceExp35cm/image09.png",
//        "img/sameDistanceExp35cm/image10.png",
//        "img/sameDistanceExp35cm/image11.png",
//        "img/sameDistanceExp35cm/image12.png",
//        "img/sameDistanceExp35cm/image13.png",
//        "img/sameDistanceExp35cm/image14.png",
//        "img/sameDistanceExp35cm/image15.png",
//        "img/sameDistanceExp35cm/image16.png",
//        "img/sameDistanceExp35cm/image17.png",
//        "img/sameDistanceExp35cm/image18.png",
//        "img/sameDistanceExp35cm/image19.png",
//        "img/sameDistanceExp35cm/image20.png",
//        "img/sameDistanceExp35cm/image21.png",
//        "img/sameDistanceExp35cm/image22.png",
//        "img/sameDistanceExp35cm/image23.png",
//        "img/sameDistanceExp35cm/image24.png",
//        "img/sameDistanceExp35cm/image25.png",
//        "img/sameDistanceExp35cm/image26.png",
//        "img/sameDistanceExp35cm/image27.png",
//        "img/sameDistanceExp35cm/image28.png",
//        "img/sameDistanceExp35cm/image29.png",
//        "img/sameDistanceExp35cm/image30.png",
//        "img/sameDistanceExp35cm/image31.png",
//        "img/sameDistanceExp35cm/image32.png",
//        "img/sameDistanceExp35cm/image33.png",
//        "img/sameDistanceExp35cm/image34.png",
//        "img/sameDistanceExp35cm/image35.png",
//        "img/sameDistanceExp35cm/image36.png",
//        "img/sameDistanceExp35cm/image37.png",
//        "img/sameDistanceExp35cm/image38.png",
//        "img/sameDistanceExp35cm/image39.png",
//        "img/sameDistanceExp35cm/image40.png",
////        "img/sameDistanceExp35cm/image41.png",
////        "img/sameDistanceExp35cm/image42.png",
//        "img/sameDistanceExp35cm/image43.png",
//        "img/sameDistanceExp35cm/image44.png",
////        "img/sameDistanceExp35cm/image45.png",
////        "img/sameDistanceExp35cm/image46.png",
//        "img/sameDistanceExp35cm/image47.png",
//        "img/sameDistanceExp35cm/image48.png",
//        "img/sameDistanceExp35cm/image49.png",
//        "img/sameDistanceExp35cm/image50.png",
//        "img/sameDistanceExp35cm/image51.png",
//        "img/sameDistanceExp35cm/image52.png",
//        "img/sameDistanceExp35cm/image53.png",
//        "img/sameDistanceExp35cm/image54.png",
//        "img/sameDistanceExp35cm/image55.png",
//        "img/sameDistanceExp35cm/image56.png",
//        "img/sameDistanceExp35cm/image57.png",
//        "img/sameDistanceExp35cm/image58.png",
//        "img/sameDistanceExp35cm/image59.png",
//        "img/sameDistanceExp35cm/image60.png",
//        "img/sameDistanceExp35cm/image61.png",
//        "img/sameDistanceExp35cm/image62.png",
//        "img/sameDistanceExp35cm/image63.png",
//        "img/sameDistanceExp35cm/image64.png",
//        "img/sameDistanceExp35cm/image65.png",
//        "img/sameDistanceExp35cm/image66.png",
//        "img/sameDistanceExp35cm/image67.png",
//        "img/sameDistanceExp35cm/image68.png",
//        "img/sameDistanceExp35cm/image69.png",
//        "img/sameDistanceExp35cm/image70.png",
//        "img/sameDistanceExp35cm/image71.png",
//        "img/sameDistanceExp35cm/image72.png",
//        "img/sameDistanceExp35cm/image73.png",
//        "img/sameDistanceExp35cm/image74.png",
//        "img/sameDistanceExp35cm/image75.png",
//        "img/sameDistanceExp35cm/image76.png",
//        "img/sameDistanceExp35cm/image77.png",
//        "img/sameDistanceExp35cm/image78.png",
//        "img/sameDistanceExp35cm/image79.png",
//        "img/sameDistanceExp35cm/image80.png",
//        "img/sameDistanceExp35cm/image81.png",
//        "img/sameDistanceExp35cm/image82.png",
//        "img/sameDistanceExp35cm/image83.png",
//        "img/sameDistanceExp35cm/image84.png",
//        "img/sameDistanceExp35cm/image85.png",
//        "img/sameDistanceExp35cm/image86.png",
//        "img/sameDistanceExp35cm/image87.png",
//        "img/sameDistanceExp35cm/image88.png",
//        "img/sameDistanceExp35cm/image89.png",
//        "img/sameDistanceExp35cm/image90.png",
//        "img/sameDistanceExp35cm/image91.png",
//        "img/sameDistanceExp35cm/image92.png",
//        "img/sameDistanceExp35cm/image93.png",
//        "img/sameDistanceExp35cm/image94.png",
//        "img/sameDistanceExp35cm/image95.png",
//        "img/sameDistanceExp35cm/image96.png",
//        "img/sameDistanceExp35cm/image97.png",
//        "img/sameDistanceExp35cm/image98.png",
//        "img/sameDistanceExp35cm/image99.png",
//        "img/sameDistanceExp35cm/image100.png",
//};
int startAt = 1;
std::vector<std::string> files = {
        "img/sameDistanceExp50cm/image01.png",
        "img/sameDistanceExp50cm/image02.png",
        "img/sameDistanceExp50cm/image03.png",
        "img/sameDistanceExp50cm/image04.png",
        "img/sameDistanceExp50cm/image05.png",
        "img/sameDistanceExp50cm/image06.png",
        "img/sameDistanceExp50cm/image07.png",
        "img/sameDistanceExp50cm/image08.png",
        "img/sameDistanceExp50cm/image09.png",
        "img/sameDistanceExp50cm/image10.png",
        "img/sameDistanceExp50cm/image11.png",
        "img/sameDistanceExp50cm/image12.png",
        "img/sameDistanceExp50cm/image13.png",
        "img/sameDistanceExp50cm/image14.png",
//        "img/sameDistanceExp50cm/image15.png",
//        "img/sameDistanceExp50cm/image16.png",
        "img/sameDistanceExp50cm/image17.png",
        "img/sameDistanceExp50cm/image18.png",
        "img/sameDistanceExp50cm/image19.png",
        "img/sameDistanceExp50cm/image20.png",
        "img/sameDistanceExp50cm/image21.png",
        "img/sameDistanceExp50cm/image22.png",
        "img/sameDistanceExp50cm/image23.png",
        "img/sameDistanceExp50cm/image24.png",
        "img/sameDistanceExp50cm/image25.png",
        "img/sameDistanceExp50cm/image26.png",
        "img/sameDistanceExp50cm/image27.png",
        "img/sameDistanceExp50cm/image28.png",
        "img/sameDistanceExp50cm/image29.png",
        "img/sameDistanceExp50cm/image30.png",
        "img/sameDistanceExp50cm/image31.png",
        "img/sameDistanceExp50cm/image32.png",
        "img/sameDistanceExp50cm/image33.png",
        "img/sameDistanceExp50cm/image34.png",
        "img/sameDistanceExp50cm/image35.png",
        "img/sameDistanceExp50cm/image36.png",
        "img/sameDistanceExp50cm/image37.png",
        "img/sameDistanceExp50cm/image38.png",
        "img/sameDistanceExp50cm/image39.png",
        "img/sameDistanceExp50cm/image40.png",
        "img/sameDistanceExp50cm/image41.png",
        "img/sameDistanceExp50cm/image42.png",
        "img/sameDistanceExp50cm/image43.png",
        "img/sameDistanceExp50cm/image44.png",
        "img/sameDistanceExp50cm/image45.png",
        "img/sameDistanceExp50cm/image46.png",
        "img/sameDistanceExp50cm/image47.png",
        "img/sameDistanceExp50cm/image48.png",
        "img/sameDistanceExp50cm/image49.png",
        "img/sameDistanceExp50cm/image50.png",
        "img/sameDistanceExp50cm/image51.png",
        "img/sameDistanceExp50cm/image52.png",
        "img/sameDistanceExp50cm/image53.png",
        "img/sameDistanceExp50cm/image54.png",
        "img/sameDistanceExp50cm/image55.png",
        "img/sameDistanceExp50cm/image56.png",
        "img/sameDistanceExp50cm/image57.png",
        "img/sameDistanceExp50cm/image58.png",
        "img/sameDistanceExp50cm/image59.png",
        "img/sameDistanceExp50cm/image60.png",
        "img/sameDistanceExp50cm/image61.png",
        "img/sameDistanceExp50cm/image62.png",
        "img/sameDistanceExp50cm/image63.png",
        "img/sameDistanceExp50cm/image64.png",
        "img/sameDistanceExp50cm/image65.png",
        "img/sameDistanceExp50cm/image66.png",
        "img/sameDistanceExp50cm/image67.png",
        "img/sameDistanceExp50cm/image68.png",
        "img/sameDistanceExp50cm/image69.png",
        "img/sameDistanceExp50cm/image70.png",
        "img/sameDistanceExp50cm/image71.png",
        "img/sameDistanceExp50cm/image72.png",
        "img/sameDistanceExp50cm/image73.png",
        "img/sameDistanceExp50cm/image74.png",
        "img/sameDistanceExp50cm/image75.png",
        "img/sameDistanceExp50cm/image76.png",
        "img/sameDistanceExp50cm/image77.png",
        "img/sameDistanceExp50cm/image78.png",
        "img/sameDistanceExp50cm/image79.png",
        "img/sameDistanceExp50cm/image80.png",
        "img/sameDistanceExp50cm/image81.png",
        "img/sameDistanceExp50cm/image82.png",
        "img/sameDistanceExp50cm/image83.png",
        "img/sameDistanceExp50cm/image84.png",
        "img/sameDistanceExp50cm/image85.png",
        "img/sameDistanceExp50cm/image86.png",
        "img/sameDistanceExp50cm/image87.png",
        "img/sameDistanceExp50cm/image88.png",
        "img/sameDistanceExp50cm/image89.png",
        "img/sameDistanceExp50cm/image90.png",
        "img/sameDistanceExp50cm/image91.png",
        "img/sameDistanceExp50cm/image92.png",
        "img/sameDistanceExp50cm/image93.png",
        "img/sameDistanceExp50cm/image94.png",
        "img/sameDistanceExp50cm/image95.png",
        "img/sameDistanceExp50cm/image96.png",
        "img/sameDistanceExp50cm/image97.png",
        "img/sameDistanceExp50cm/image98.png",
        "img/sameDistanceExp50cm/image99.png",
        "img/sameDistanceExp50cm/image100.png",
        "img/sameDistanceExp50cm/image101.png",
        "img/sameDistanceExp50cm/image102.png",
};
int main(int argc, char **argv) {

    if (!startFaceTracker()) {
        return 0;
    };
    if (!startCapture()) {
        return 0;
    };


    int i = startAt - 1;

    while (i + 1 < files.size()) {
        cv::Mat A, B;

        std::string fA = files[i];
        std::string fB = files[i + 1];
        i += 2;

        cout << fA << endl;
        cout << fB << endl;

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
//        cv::waitKey(10);


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
//    tracker->Reset();
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

int c = 0;

void logMeasurements(ReturnedPoints data) {

    auto pointsFace = std::get<0>(data);
    auto leftRawPoint = std::get<1>(data);
    auto rightRawPoint = std::get<2>(data);
    vector<double> corrected3d;
    vector<double> rawLeft;
    vector<double> rawRight;
    //smile
    corrected3d.push_back(fabs(pointsFace[54].x - pointsFace[48].x));
    rawLeft.push_back(fabs(leftRawPoint[54].x - leftRawPoint[48].x));
    rawRight.push_back(fabs(rightRawPoint[54].x - rightRawPoint[48].x));

    vector<string> names = {
            "smile"
    };

    cout << "======================" << endl;
    std::cout << "====" << c << "====" << std::endl;
    cout << "======================" << endl;
    std::cout << "corrected | raw left | raw right" << endl;
    c++;
    for (int i = 0; i < corrected3d.size(); i++) {
        cout << names[i] << "\t" << corrected3d[i] << "\t" << rawLeft[i] << "\t" << rawRight[i] << endl;
    }
    cout << "======================" << endl;
    cout << "======================" << endl;


}


