//
// Created by juarez on 22/09/16.
//

#ifndef BLENDSHAPESVIEWER_UTIL_H
#define BLENDSHAPESVIEWER_UTIL_H

#include <tracker/FaceTracker.hpp>
#include <opencv2/highgui.hpp>
#include "global.h"
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

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

void startFaceTracker(FACETRACKER::FaceTracker **trackerToStart, FACETRACKER::FaceTrackerParams **params) {
    *trackerToStart = FACETRACKER::LoadFaceTracker();
    *params = FACETRACKER::LoadFaceTrackerParams();

    if (*trackerToStart == nullptr) {
        std::cerr << "Cannot load tracker!" << std::endl;
        return;
    }

    if (*params == nullptr) {
        std::cerr << "Cannot load tracker params!" << std::endl;
        return;

    }

    return;
}

bool startCapture() {
    capture.at(0).open("video/out_mouth1_2.avi");
    capture.at(1).open("video/out_mouth2_2.avi");

//    capture.at(0).open("video/out_eyebrow1_1.avi");
//    capture.at(1).open("video/out_eyebrow2_1.avi");

//    capture.at(0).open("video/out_happy1_1.avi");
//    capture.at(1).open("video/out_happy2_1.avi");
//
//    capture.at(0).open("video/out_eyes1_1.avi");
//    capture.at(1).open("video/out_eyes2_1.avi");

    if (!capture.at(0).isOpened() || !capture.at(1).isOpened()) {
        std::cerr << "cannot open camera!" << std::endl;
        return false;
    }

    return true;
}

void setFilterConfigurations(double **coefficients, int *sizeOfFilter, std::vector<std::string> &filterDescription) {
    coefficients[0] = new double[1]{1.0};
    sizeOfFilter[0] = 1;
    filterDescription.push_back("no filter at all");

    coefficients[1] = new double[3]{0.25, 0.5, 0.25};
    sizeOfFilter[1] = 3;
    filterDescription.push_back("hanning filter order 1");

    coefficients[2] = new double[4]{0.64f, 0.16f, 0.16f};
    sizeOfFilter[2] = 3;
    filterDescription.push_back("filtro inventado pelo rodrigo");

    // wc=0.1 pi
    coefficients[3] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.1pi");

// wc =     0.188888888888889 pi
    coefficients[4] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.19pi");

//            0.277777777777778

    coefficients[5] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.28pi");


//            0.366666666666667 pi

    coefficients[6] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.37pi");

//            0.455555555555556 pi

    coefficients[7] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.45pi");

//            0.544444444444444 pi

    coefficients[8] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.54pi");

//            0.633333333333333 pi

    coefficients[9] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.63pi");

//            0.722222222222222

    coefficients[10] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.72pi");

//            0.811111111111111 pi

    coefficients[11] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.81pi");

//            0.900000000000000 pi

    coefficients[12] = new double[17]{
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
    filterDescription.push_back("low pass, wc = 0.9pi");

}

#include <string>
#include <tuple>
#include <vector>

namespace fs = boost::filesystem;

std::vector<std::pair<std::string, std::string>> getFilesOnDir(std::string path) {
    fs::path targetDir(path.c_str());

    fs::directory_iterator it(targetDir), eod;

    std::map<std::string, int> allFiles;

    BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod)) {
                    if (fs::is_regular_file(p) && (fs::extension(p).compare(".png") == 0)) {
                        // do something with p
                        allFiles.insert(std::make_pair(p.c_str(), 0));
                    }
                }

    std::vector<std::string> listOfFiles;

    for (auto it : allFiles) {
        listOfFiles.push_back(it.first);
    }

    std::vector<std::pair<std::string, std::string>> output;

    for (int i = 0; i + 1 < listOfFiles.size(); i += 2) {
        output.push_back(std::make_pair(listOfFiles[i], listOfFiles[i + 1]));
    }

    return output;

}

#endif //BLENDSHAPESVIEWER_UTIL_H
