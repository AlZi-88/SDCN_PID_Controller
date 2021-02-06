# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Writeup
The target of this project is to code a PID controller which controls a car around a circuit without driving off the road. The implementation of the PID controller is quite straight forward. As the name already implies the controller consists of a proportional, integral and derivative part. My implementation uses such kind of controller to set the steering angle of the car. It uses therefore the control error `cte`, which is the difference of the car to the center of the road. The controlled steering angle calculates then as follows:
```math
steer = - K_p * cte - K_i * cte_sum
- K_d * (cte - cte_old)
```

With `cte_sum` as integrated error `cte_sum += cte` and `cte_old` as error from previous iteration.

What remains is to find the right parameters Kp, Ki and Kd which lead to a stable control of the vehicle. My first approach to find these parameters was to randomly chose the parameters which did not succeed at all, the parameters I had chosen were much too high so that the steering was only oscillating between min and max. When I observed then the values of error itself, the integral value as well as the derivative I saw that I have to put much slower values for integral value as to the others since the integral value could become very big very fast. I also saw that for Kp value it makes no sense to have values grater than 0.333 since the boarder of the road has an error of cte ~= 3.0 and the maximum steering angle is 1.0. So to have at maximum error the maximum steering angle, I have to muliply the error by 0.333.

With these initial parameters I was at least able to somehow able to finish one lap of the track but the result was not satisfying at all. The car was behaving quite well in curves but on the straight parts of the track it was oscillating a lot. So I decided to add a twiggle algorithm which adjusts the parameters dependent on their performance. The algorithm increases the parameters of the controller and checks if the performance is increasing or not. For those parameters, the performance gets worse the algorithm tries if decreasing brings any benefit. The evaluation is done after the car has finished one lap, or when ever the car has an accident. The criterias for the performance are the average error `error_av` over the whole distance and the distance itself (which is basically the number of iterations of the algorithm). The algorithm stops to find a optimum when the sum of all changes on the parameters is below a threshold. I had chosen 1E-10 to allow a sufficient long optimization but I guess a slightly higher value ~1E-7 - 1E-8 would also be sufficient.

With this algorithm finally I was able to find acceptable parameters which control the car quite smoothly around the track. These are the parameters I finally applied:

|      Kp      |        Ki       |      Kd     |
|--------------|-----------------|-------------|
|   0.191657   |   0.000401418   |   3.70017   |
