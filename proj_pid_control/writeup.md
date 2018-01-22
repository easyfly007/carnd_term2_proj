# CarND PID controller project

## project basic information

this is the project code for Udacity CarND (nano degree) term2 PID control project.
the purpose of the project is to provide an algorithm that result a proper steering and throttle value based on the received speed and cross ctrack error (cte).

when the server code provided a proper throttle and steer value, the simulation car will response accordingly. (with some delay).

## steering algorithm

the main algorithm is to use PID control, while:

* P: Proportional term, e.g., the cte value (offset to the reference)
* D: differential term, e.g., the cte difference between this time point and last time point
* I: integral term, e.g., the accumulated cte value from the begining.

then we need to provide 3 hyper parameters for the control, Kp, Kd, Ki.

the final steering value will be like  
```
steer_value = - P * Kp - D * Kd - I *Ki 
```

these 3 terms plays different rols in the control step.

* P: will switch the car from offset to the center, enven cross the center line. make the car swing.
* D: will make the car move smooth, prohibit the car to move swing so much.
* I: P and D will provide a smooth routine for the car, but cannot remove the systematic offset of the car. in that condition, I will take the systematic offset effect and provide revise.


## throttle algorithm

* when throttle > 0.0, the car will try to speed up;
* when throttle < 0.0, the car will do a break and then move backward.
* when throttle = 0.0, the car will move slower and slower.

in my observation, the car will loss speed when turn sround or driving on the uneven road.
so in most of the condition, we should give the throttle a small positive value to make the car move ahead.

in my code, I give 0.1 in most of the conditions.

when in the car turn a round and attemp to dive out of the road, I may need a break to give more control in a short road distance.

## hyper parameters and more tunning

in my final solution, I use Kp, Ki, Kd = 0.1, 0.0003, 0.6.
the difficult one is that the road after a bridge, the turn is ver sharp and may drive the car out of the road.

I give a small tunning that in condition the car seems to drive out of the road, make the Kd large.
