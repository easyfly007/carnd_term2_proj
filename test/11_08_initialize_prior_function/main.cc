#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

//initialize priors assumimg vehicle at landmark +/- 1.0 meters position stdev
std::vector<float> initialize_priors(int map_size, std::vector<float> landmark_positions,
                                     float control_stdev);

int main() {

    //set standard deviation of position:
    float control_stdev = 1.0f;


    //set map horizon distance in meters 
    int map_size = 25;

    //initialize landmarks
    std::vector<float> landmark_positions {5, 10, 20};

    // initialize priors
    std::vector<float> priors = initialize_priors(map_size, landmark_positions,
                                                  control_stdev);
    
    //print values to stdout 
    for (unsigned int p = 0; p < priors.size(); p++) {
        std::cout << priors[p] << endl;
    }
        
    return 0;

};

//TODO: Complete the initialize_priors function
std::vector<float> initialize_priors(int map_size, std::vector<float> landmark_positions,
                                     float control_stdev) {
    
    uint positive_pos = uint(landmark_positions.size() * (1.0 + 2 * control_stdev));
    float p_pos = 1.0 / float(positive_pos);
    std::vector<float> v(0.0, map_size);
    for (size_t i = 0; i < landmark_positions.size(); i ++)
    {
        float pos = landmark_positions[i];
        for (float j = pos - control_stdev; j <= pos + control_stdev; j += 1.0)
            v[int(j)] = p_pos;
    }
    return v;



//initialize priors assumimg vehicle at landmark +/- 1.0 meters position stdev

    //YOUR CODE HERE

    return v;
}
