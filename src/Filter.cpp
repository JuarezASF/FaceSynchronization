//
// Created by juarez on 22/09/16.
//

#include "Filter.h"

HanningFilter::HanningFilter() {
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;
}

float HanningFilter::updateValue(float sample) {
    x[0] = x[1];
    x[1] = x[2];
    x[2] = sample;

    return 0.25f * (x[2] + 2 * x[1] + x[0]);


}
