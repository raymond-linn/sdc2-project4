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
As following Youtube video, we can see that only with P - Controller (Kp), it gives the Steering angle unstable and in seconds of driving, the car is off track.

[![Alt p only](https://github.com/raymond-linn/sdc2-project4/blob/master/p-only.png)](https://youtu.be/xrQiyPdVDkQ)

##### P and D (Derivatives) Control
After adding the Derivatives control and run the simulation again and I get the pretty stable driving. This D - Controller (Kd) determines the slope of the error so it is much stable. 

[![Alt p only](https://github.com/raymond-linn/sdc2-project4/blob/master/p-only.png)](https://youtu.be/n9njXUNhq3Y)

##### I - Integral Control
The integral is proportional to the magnitude of the error and the duration of the error. After I added the I - Controller (Ki), the simulation seems more smoother. You can increase the value of what I got as optimzed Integral parameter to see eratic movement of the car in a second since it overshoots in adjustment. 

Finally with Integral Control, I get to a better parameters that could drive on the track. My parameters that run in the following video comes down to be Kp = 0.15, Ki = 0.001, Kd = 3.0. They might not be the best one but for this moment they do the job to satisfy the project requirements.

[![Alt p only](https://github.com/raymond-linn/sdc2-project4/blob/master/p-only.png)](https://youtu.be/FzTkLmZ4ivg)

This project has been a challenge and fun in tuning the parameters. I have been researching to see what other tuning methods rather than manual, twiddle, SGD and Ziegler-Nichols method. Trying to find out whether is there any paper on the comparison of methods advantages and disadvantages on the real application. It is for my future study. 
