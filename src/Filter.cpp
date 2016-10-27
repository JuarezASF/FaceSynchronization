//
// Created by juarez on 22/09/16.
//

#include "Filter.h"

HanningFilter::HanningFilter() {
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;
    v[0] = 1;
    v[1] = 2;
    v[2] = 1;
    v[3] = 0.25f;
}


void HanningFilter::updateP(float a, float b, float c, float d) {
    v[0] = a;
    v[1] = b;
    v[2] = c;
    v[3] = d;
}

float HanningFilter::updateValue(float sample) {
    x[0] = x[1];
    x[1] = x[2];
    x[2] = sample;

    return v[3] * (v[2] * x[2] + v[1] * x[1] + v[0] * x[0]);


}
