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

WindowFilter::WindowFilter(int sizeOfFilter, float *coefficients) {

    this->sizeOfFilter = sizeOfFilter;
    this->coefficients = coefficients;
    this->x = new float[sizeOfFilter];

}

float WindowFilter::updateValue(float sample) {
    x[0] = x[1];
    x[1] = x[2];
    x[2] = sample;

    for(int i = 0; i < sizeOfFilter - 1; i++ )
        x[i] = x[i+1];

    x[sizeOfFilter - 1] = sample;

    float output = 0;

    for(int i = 0; i < sizeOfFilter - 1; i++ )
        output += x[i] * coefficients[i];

    return output;
}

WindowFilter::~WindowFilter() {
    delete[] this->x;
}





