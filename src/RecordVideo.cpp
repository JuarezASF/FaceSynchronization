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

bool track(cv::Mat &frame, string windowName, FACETRACKER::FaceTracker *tracker,
           FACETRACKER::FaceTrackerParams *params);

bool readAndExecuteOnce();

//frame is used for web cam input; but the CSIRO face detection works only on gray scale images

//camera parameters
double f1 = (854.792781659906610 + 857.614193372593600) / 2;
double f2 = (834.378121568220080 + 835.838850269775660) / 2;
double b = 18;

namespace fs = boost::filesystem;

std::fstream f;

VideoCapture cap1, cap2;

FACETRACKER::FaceTracker **trackerA, **trackerB;
FACETRACKER::FaceTrackerParams **paramsA, **paramsB;

Mat frame1, frame2;

string imageNameA, imageNameB;

int main(int argc, char **argv) {

    char dirName[100];
    char experimentName[100];

    printf("Enter name of directory to save output:");
    scanf("%s", experimentName);
    getchar();

    sprintf(dirName, "output/%s", experimentName);

    boost::filesystem::path dir(dirName);
    if (boost::filesystem::create_directory(dir)) {
        std::cout << "Directory Created: " << string(dirName) << std::endl;
    } else {
        std::cerr << "Error while creating directory:" << string(dirName) << endl;
        return 0;
    }

    trackerA = new FACETRACKER::FaceTracker *;
    trackerB = new FACETRACKER::FaceTracker *;

    paramsA = new FACETRACKER::FaceTrackerParams *;
    paramsB = new FACETRACKER::FaceTrackerParams *;

    startFaceTracker(trackerA, paramsA);
    if (*trackerA == nullptr || *paramsA == nullptr) {
        cerr << "cannot start tracker A" << endl;
        return 0;
    }

    startFaceTracker(trackerB, paramsB);
    if (*trackerB == nullptr || *paramsB == nullptr) {
        cerr << "cannot start tracker B" << endl;
        return 0;
    }


    cap1.open(1);
    cap2.open(2);

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

    int mode = 0;

    int toCapture = 40;
    int captured = 0;

    int waitTime = 30;

    while (true) {
        cap1 >> frame1;
        cap2 >> frame2;

        imshow("InputA", frame1);
        imshow("InputB", frame2);

        char q = (char) (0xFF & waitKey(waitTime));
        if (mode == 0) {
            if (q == 27) {
                cout << "esc key is pressed by user" << endl;
                break;
            }

            if (q == 's') {
                waitTime = 100;
                mode = 1;
            }

        }
        else if (mode == 1) {

            if (captured < toCapture) {
                if (readAndExecuteOnce()) {
                    //write frames to file
                    char nameABuffer[100];
                    char nameBBuffer[100];

                    sprintf(nameABuffer, "output/%s/image%02dL.png", experimentName, captured);
                    imwrite(nameABuffer, frame1);

                    sprintf(nameBBuffer, "output/%s/image%02dR.png", experimentName, captured);
                    imwrite(nameBBuffer, frame2);

                    printf("image# %02d captured. Saved to %s and %s\n", captured, nameABuffer,
                           nameBBuffer);

                    captured++;
                } else {

                }

            }
            else {
                waitTime = 30;
                mode = 2;
            }
        }
        else {
            cout << "All frames were captured!" << endl;
            break;
        }


    }

    return 0;


}

bool readAndExecuteOnce() {

    bool sucess = track(frame1, "TrackedA", *trackerA, *paramsA);
    sucess &= track(frame2, "TrackedB", *trackerB, *paramsB);

    return sucess;


}

int cameraNum = 0;


bool track(cv::Mat &frame, string windowName,
           FACETRACKER::FaceTracker *tracker, FACETRACKER::FaceTrackerParams *params) {
    std::vector<cv::Point_<double> > points;
    cv::Mat grayScale;

    if (frame.empty()) {
        std::cout << "Empty image received!" << std::endl;
        return false;
    }

    cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);

    //update state of tracker with new frame
    int detectionQuality = tracker->NewFrame(grayScale, params);

    if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
        tracker->Reset();
        detectionQuality = tracker->NewFrame(grayScale, params);

        if (detectionQuality == 0 || detectionQuality == FACETRACKER::FaceTracker::TRACKER_FAILED) {
            std::cerr << "error while tracking! problematic frame" << windowName << std::endl;
            return false;
        }
    }

    //obtain points that were tracked
    points = tracker->getShape();

    for (auto p : points) {
        cv::circle(grayScale, p, 3, cv::Scalar(0, 0, 255), 1);
    }

    imshow(windowName, grayScale);
    cv::waitKey(5);


    return true;
}




