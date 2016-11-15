#include <algorithm>
#include <iostream>
#include <fstream>

#include <iostream>
#include <opencv2/opencv.hpp>


#include "tiny_obj_loader.h"

#include "global.h"
#include "util.h"


#include <mutex>

using namespace cv;
using namespace std;

std::vector<cv::Point_<double> > track(cv::Mat &frame, string windowName, FACETRACKER::FaceTracker *tracker,
FACETRACKER::FaceTrackerParams *params);

//frame is used for web cam input; but the CSIRO face detection works only on gray scale images

//camera parameters
double f1 = (854.792781659906610 + 857.614193372593600) / 2;
double f2 = (834.378121568220080 + 835.838850269775660) / 2;
double b = 18;

namespace fs = boost::filesystem;

std::fstream f;


int main(int argc, char **argv) {

    VideoCapture cap1, cap2;

    FACETRACKER::FaceTracker **trackerA, **trackerB;
    FACETRACKER::FaceTrackerParams **paramsA, **paramsB;

    trackerA = new FACETRACKER::FaceTracker*;
    trackerB = new FACETRACKER::FaceTracker*;

    paramsA = new FACETRACKER::FaceTrackerParams*;
    paramsB = new FACETRACKER::FaceTrackerParams*;


    startFaceTracker(trackerA, paramsA);
    if(*trackerA == nullptr || *paramsA == nullptr){
        cerr << "cannot start tracker A" << endl;
        return 0;
    }

    startFaceTracker(trackerB, paramsB);
    if(*trackerB == nullptr || *paramsB == nullptr){
        cerr << "cannot start tracker B" << endl;
        return 0;
    }


    cap1.open(1);
    cap2.open(2);

    Mat frame1, frame2;

    string imageNameA, imageNameB;
    int imageNumber = 1;

    cv::namedWindow("InputA", CV_WINDOW_FREERATIO);
    cv::namedWindow("InputB", CV_WINDOW_FREERATIO);

    cv::namedWindow("TrackedA", CV_WINDOW_FREERATIO);
    cv::namedWindow("TrackedB", CV_WINDOW_FREERATIO);

    cv::moveWindow("InputA", 0, 0);
    int width = 350;
    cv::moveWindow("InputB", 0, width);

    cv::moveWindow("TrackedA", width, 0);
    cv::moveWindow("TrackedB", width, width);

    cv::resizeWindow("InputA", width, width);
    cv::resizeWindow("InputB", width, width);

    cv::resizeWindow("TrackedA", width, width);
    cv::resizeWindow("TrackedB", width, width);


    while (true) {
        cap1 >> frame1;
        cap2 >> frame2;

        imshow("InputA", frame1);
        imshow("InputB", frame2);

        track(frame1, "TrackedA", *trackerA, *paramsA);
        track(frame2, "TrackedB", *trackerB, *paramsB);


        char q = (char) (0xFF & waitKey(30));
        if (q == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }

        if (q == 'p') //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            imageNameA = "output/image" + to_string(imageNumber) + "L.png";
            imwrite(imageNameA, frame1);

            imageNameB = "output/image" + to_string(imageNumber) + "R.png";
            imwrite(imageNameB, frame2);

            printf("image# %02d captured. Saved to %s and %s\n", imageNumber, imageNameA.c_str(), imageNameB.c_str());
        }

        if (q == 'c') //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            imageNumber += 1;
            cout << "next" << endl;
        }

    }

    return 0;


}

int cameraNum = 0;


std::vector<cv::Point_<double> > track(cv::Mat &frame, string windowName,
    FACETRACKER::FaceTracker *tracker, FACETRACKER::FaceTrackerParams *params) {
    std::vector<cv::Point_<double> > points;
    cv::Mat grayScale;

    if (frame.empty()) {
        std::cout << "Empty image received!" << std::endl;
        return points;
    }

    cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);

    //update state of tracker with new frame
    int detectionQuality = tracker->NewFrame(grayScale, params);

    if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
        tracker->Reset();
        detectionQuality = tracker->NewFrame(grayScale, params);

        if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
            std::cerr << "error while tracking! problematic frame" << windowName << std::endl;
            return points;
        }
    }

    //obtain points that were tracked
    points = tracker->getShape();

    for (auto p : points) {
//            putText(frame, std::to_string(count), p, 1, 1, cv::Scalar(255, 0, 0));
        cv::circle(grayScale, p, 3, cv::Scalar(0, 0, 255), 1);
    }

    imshow(windowName, grayScale);
    cv::waitKey(5);

    cameraNum++;


    return points;

}




