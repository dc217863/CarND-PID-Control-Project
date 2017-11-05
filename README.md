# CarND-Controls-PID

Self-Driving Car Engineer Nanodegree Program

---

## PID Controller

### P - Proportional

The P (proportional) component -Kp * CTE of the PID controller reacts directly to the cross track error (CTE). A huge CTE leads to strong steering intervention. If the Kp value is too large the controller overshoots the center line and starts to oscillate. If Kp is too low the controller reacts slowly on cross track errors. The vehicle tends to leave the track especially in sharp curves.

### I - Integral

The I (integral) component Ki * sum(cte) * dt of the PID controller reduces constant bias and drifts from the center line. Large Ki values leads to an oscillating controller.

### D - Differential

The D (differential) component Kd * (CTE - prev_CTE) / dt of the PID controller reduces the overshoots caused by the P component. It acts as a kind of damping mechanism. Small Kd values do not damp the system thus the controller tends to overshoot and oscillation. Too large Kd values leads to a slow reduction of the cross track error.

## PID Tuning Process

To tune the steering angle PID controller manually I applied the following steps.

1. Set all values (Kp, Ki, Kd) to zero.
2. Increase the Kp value until the controller starts to oscillate.
3. Increase the Kd value to damp the oscillation until the vehicle is able to drive one full round on the race track. In case the oscillation is still to high, slightly reduce the Kp value.
4. Finally increase the Ki value in small steps to reduce the bias and drifts especially incurves.

Only manual tuning was carried out as it gives a better feel of the values being changed. Another approach explained in the course is Twiddle to automatically tune the parameters.

### Final PID Values

* Kp = 0.09
* Ki = 0.00001
* Kd = 0.9

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.

    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Reflections

The solution achieved here with PID controllers is not a viable solution for self driving cars in general. This is an overfit solution tested only in the Udacity simulator.

An adaptive controller that can change its control parameters based on the situation would be much more suitable.
