/*
License: MIT License (http://www.opensource.org/licenses/mit-license.php)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/* (C) 2013 Graeme Hattan & Bernd Porr */

#include "FIR.h"

#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <cassert>

Fir1::Fir1(double *coefficients, unsigned number_of_taps) :
	coefficients(coefficients),
	// addition of brackets should mean the array is 
	// value-initialised to be all zeros
	buffer(new double[number_of_taps]()),  
	taps(number_of_taps),
	offset(0)
{
	reset();
}


Fir1::~Fir1()
{
  delete[] buffer;
//  delete[] coefficients;
}

double Fir1::filter(double input)
{
	double *coeff     = coefficients;
	double *coeff_end = coefficients + taps;

	double *buf_val = buffer + offset;

	*buf_val = input;
	double output_ = 0;
	
	while(buf_val >= buffer)
		output_ += *buf_val-- * *coeff++;
	
	buf_val = buffer + taps-1;
	
	while(coeff < coeff_end)
		output_ += *buf_val-- * *coeff++;
	
	if(++offset >= taps)
		offset = 0;
	
	return output_;
}

void Fir1::reset()
{
	memset(buffer, 0, sizeof(double)*taps);
	offset = 0;
}

void Fir1::updateCoefficients(double *coefficients, int numberOfTaps) {
	delete[] buffer;
	this->coefficients = coefficients;
    buffer = new double[numberOfTaps];
    taps = (unsigned int) numberOfTaps;
	this->offset = 0;
	reset();

}


