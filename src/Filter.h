//
// Created by juarez on 22/09/16.
//

#ifndef BLENDSHAPESVIEWER_FILTER_H
#define BLENDSHAPESVIEWER_FILTER_H


class Filter {

public:

    virtual double updateValue(float sample) = 0;

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

    double updateValue(float sample) override;


};

class WindowFilter : public Filter {
private:
    int sizeOfFilter;
    double *x, *coefficients;
public:

    WindowFilter(int sizeOfFilter, double *coefficients);

    WindowFilter();

    virtual ~WindowFilter();

    double updateValue(float sample) override;

    void construct(int sizeOfFilter, double *coefficients);

    void updateCoefficients(int size, double *coeff);


};


#endif //BLENDSHAPESVIEWER_FILTER_H
