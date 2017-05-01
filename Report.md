### PID Project Report

---

#### Parameter Optimization
As discussed in Lesson 16, lecture 14 and 15, I tried to use twiddle algorithm for optimizing the parameters of P, I and D. 

Turning on the is_opt_pram_on to be true and run the program to print out the best errors with parameter values. It was a lot of manual work since using p and dp as in the lesson, simulating car always crashes into the land after looping around on the dirt.

And it also if I don't initialize the parameters while running twiddle algorithm, it gets crashed nearby the bridge and could not recover to be on the track.

It turned out the best error fluctuates and it seems like some better tuning method might need to be applied with this current simulation data.

#### Reflection on the Project
##### P - Proportional Control
Proportional term in PID produces an output value that is proportional to the current road value. 
As following Youtube video, we can see that only with P - Controller, it gives the Steering angle unstable and in seconds of driving, the car is off track.

[![Alt p controller only](https://www.youtube.com/upload_thumbnail?v=xrQiyPdVDkQ&t=1&ts=1493663745846)](https://youtu.be/xrQiyPdVDkQ)
