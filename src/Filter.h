//
// Created by juarez on 22/09/16.
//

#ifndef BLENDSHAPESVIEWER_FILTER_H
#define BLENDSHAPESVIEWER_FILTER_H


class Filter {

public:

    virtual float updateValue(float sample) = 0;

};

/**
 * reference: http://www.robots.ox.ac.uk/~sjrob/Teaching/SP/l6.pdf
 */
class HanningFilter : public Filter {
private:
    /**
     * x[0] is the oldest sample point x[k - 2]
     * x[3] is the most recent sample point x[k]
     */
    float x[3];
    float v[4];
public:

    HanningFilter();

    void updateP(float a, float b, float c, float d);

    float updateValue(float sample) override;


};

class WindowFilter : public Filter {
private:
    int sizeOfFilter;
    float *x, *coefficients;
public:

    WindowFilter(int sizeOfFilter, float *coefficients);

    WindowFilter();

    virtual ~WindowFilter();

    float updateValue(float sample) override;

    void construct(int sizeOfFilter, float *coefficients);

    void updateCoefficients(int size, float *coeff);


};


#endif //BLENDSHAPESVIEWER_FILTER_H
