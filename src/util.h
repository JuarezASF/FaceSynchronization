//
// Created by juarez on 22/09/16.
//

#ifndef BLENDSHAPESVIEWER_UTIL_H
#define BLENDSHAPESVIEWER_UTIL_H

#include <tracker/FaceTracker.hpp>
#include <opencv2/highgui.hpp>
#include "global.h"

bool startFaceTracker() {
    bool success = true;
    tracker = FACETRACKER::LoadFaceTracker();
    params = FACETRACKER::LoadFaceTrackerParams();

    if (tracker == nullptr) {
        std::cerr << "Cannot load tracker!" << std::endl;
        success = false;
    }

    if (params == nullptr) {
        std::cerr << "Cannot load tracker params!" << std::endl;
        success = false;

    }

    return success;
}

bool startCapture(){
    capture.at(0).open(1);
    capture.at(1).open(2);

    if (!capture.at(0).isOpened() || !capture.at(1).isOpened()) {
        std::cerr << "cannot open camera!" << std::endl;
        return false;
    }

    return true;
}

#endif //BLENDSHAPESVIEWER_UTIL_H
