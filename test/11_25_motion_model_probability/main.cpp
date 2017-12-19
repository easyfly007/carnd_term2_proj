//=================================================================================
// Name        : help_functions.h
// Version     : 2.0.0
// Copyright   : Udacity
//=================================================================================

#include <iostream>
#include "help_functions.h"

//TODO: assign a value, the difference in distances between x_t and x_{t-1}
//for an x of 7 and a pre-pseudo position of 5

float value = ?;//YOUR VALUE HERE


float parameter = 1.0; //set as control parameter or observation measurement
float stdev = 1.0; //position or observation standard deviation

int main() {

    float prob = Helpers::normpdf(value, parameter, stdev);

    std::cout << prob << endl;

    return 0;
}