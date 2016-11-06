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
    this->x = nullptr;
    this->coefficients = nullptr;
    construct(sizeOfFilter, coefficients);
}

float WindowFilter::updateValue(float sample) {
    for (int i = 0; i < sizeOfFilter - 1; i++)
        x[i] = x[i + 1];

    x[sizeOfFilter - 1] = sample;

    float output = 0;

    for (int i = 0; i < sizeOfFilter; i++)
        output += x[i] * coefficients[i];

    return output;
}

WindowFilter::~WindowFilter() {
    if (this->x != nullptr)
        delete this->x;
}

void WindowFilter::construct(int sizeOfFilter, float *coefficients) {
    if (this->x != nullptr)
        delete this->x;

    this->sizeOfFilter = sizeOfFilter;
    this->coefficients = coefficients;
    this->x = new float[sizeOfFilter];

    for (int i = 0; i < sizeOfFilter; i++)
        this->x[i] = 0.0f;

}

WindowFilter::WindowFilter() {
    this->x = nullptr;
    this->coefficients = nullptr;
}

void WindowFilter::updateCoefficients(int size, float *coeff) {
    this->sizeOfFilter = size;

    if (this->x != nullptr)
        delete this->x;

    this->x = new float[size];

    this->coefficients = coeff;
    for (int i = 0; i < sizeOfFilter; i++)
        this->x[i] = 0.0f;

}











