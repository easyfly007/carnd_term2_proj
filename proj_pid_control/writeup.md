# CarND PID controller project

## project basic information

this is the project code for Udacity CarND (nano degree) term2 PID control project.
the purpose of the project is to provide an algorithm that result a proper steering value based on the received speed and cross ctrack error (cte).

when the server code provided a proper throttle, the simulation car will response accordingly. (with some delay).

## steering algorithm

the main algorithm is to use PID control, while:

* P: Proportional term, e.g., the cte value (offset to the reference)
* D: differential term, e.g., the cte difference between this time point and last time point
* I: integral term, e.g., the accumulated cte value from the begining.

then we need to provide 3 hyper parameters for the control, Kp, Kd, Ki.

the final steering value will be like  below.
here I suppose the Kp, Kd, Ki are all positive.
if remove the '-' in the equation, the Ki, Kp, Kd should be negative value,
but actually they are the same.

```
steer_value = - P * Kp - D * Kd - I *Ki 
```

these 3 terms plays different rols in the control step.

* P: will switch the car from offset to the center, enven cross the center line. make the car swing.
* D: will make the car move smooth, prohibit the car to move swing so much.
* I: P and D will provide a smooth routine for the car, but cannot remove the systematic offset of the car. in that condition, I will take the systematic offset effect and provide revise.


## throttle algorithm

* I just give throttle 0.1 as constant value.

## hyper parameters and more tunning

in my final solution, I use Kp, Ki, Kd = 0.2, 0.0003, 0.7.

how to get this result?

*. firstly I will keep Ki, Kd as 0.0 and tune Kp.

for Kp, it will try to rotate the car to the road center direction, 

if I use one as large as 1, then it will somehow roate too much and will cross the road center.

if I use one as smaller as 0.01, then in the road turning around condition, it will not provide enough steering and the car will run out of the road.

this tells me that maybe a suitable Kp will be at 0.1 ~ 0.5 such level.

*. then after set the Kp, I begin to tune Kd.

I think Kd plays an important role which will try to make the car roate smooth,

e.g., when Kp try to rotate the car from road side to road center, Kd will do a conpensation that make the roate not so much to avoid cross the road center.
I find that a proper Kd will be like 0.7 is OK. (I tested several values)


*. then comes the Ki.

in the ideal condition, the car will follow the road center with Kp and Kd.
(Kp make the car follow the line, Kd make the car smooth)
but after a long time run on the straight road (like on the bridge), the car is not on the center of the road, it always has a ~ +0.3 offset.

that is the systematic offset, so I add the Ki to remove it.
I begin the Ki from a very small value, like 0.00001, and check if the offset is removed.
if not, I will slightly increase it.

finally I got the solution of Kp, Ki, Kd as  0.2, 0.0003, 0.7, by such PID controller, my car can perfectly run through!!