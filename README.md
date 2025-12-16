# AMR-Project-Anti-Ackerman-
Aidan Fitzpatrick AMR Final Project

Why? 
Anti-Ackerman steering, is the drive platform used in Formula 1 cars, a high-performance speed and efficiency vehicle, in which the outermost (front) wheel is steered slightly more than the innermost (front) wheel on turns. This is employed to allow for the most amount of grip and thus speed performance. Formula 1 is a field I find very interesting and relevant to this course, as it can be considered the pinnacle of engineering and performance under strict guidelines and restrictions, much like autonomous driving and robots.

Traditional Ackerman steering is used in regular commercial cars and minimizes tire slip, thus limiting wear on the tires and increasing longevity. However, this limits performance, as a tire has to have a lateral slippage in order to create a cornering force. If there is more slippage (slip angle) on the outermost tire, than the innermost, there will be more grip on the car and the vehicle will turn faster and more efficiently (at the cost of tire wear). This is achieved by having the outer tire turn more than the inner.

Pros of Anti-Ackerman:
Optimization of Slip-Angles at High-Speed
Better Cornering Force

Pros of 4 Wheel:
Easier modelling and more practical
Sufficient stability
Option to introduce suspension variable 

4-Wheel Anti-Ackerman: Operational Design Domain:
Roadway Type: Flat, hard, smooth, dry surface with no elevation changes (2-dimensional plane/planar motion)
Vehicle Configuration: 4-wheels, outer (front) wheel turn angle is larger than inner (front) wheel turn angle
Wheel Dynamics: No-slip condition (rolling without slip)
Chassis Dimensions: Known track widths and wheel bases






kinematics_validation.mlx
The following are my MATLAB plots. For case 1 (left) I kept a Vx as a constant 10.0 m/s and Vy as 1.0 m/s, and calculated omega at each time step. For case 2 (right) everything was kept the same, but I set Vx to linearly increase from 0-10 over the same time step. The steering profile was set to a simple sinusoidal pattern, where the outside wheel angle was set to 1.2x the inside wheel angle. The maximum angle of steer was set to 25 degrees.


dynamics_validation.mlx
The following are my MATLAB plots from calculating the dynamics of my vehicle using its retrieved EOMs. I kept the same artificial sinusoidal steering profile as in the kinematics modeling matlab code and kept a force/torque profile of a constant 4000N on just the rear two wheels, as Formula 1 cars are typically rear wheel drive. The plots below show the trajectory and velocities (left two plots) that occur due to the inputted sinusoidal steering profile (right)


controller.mlx
I opted for a robust feedback linearization controller because my dynamics are non-linear (yaw, lateral forces, steering coupling). It tries to control the yaw rate and forward velocity via the rear wheel forces (F3, F4) as the steering profile is kept the same sinusoidal pattern. The desired yaw rate is a smooth sine wave and the desired forward velocity is 8m/s to try and simulate a smooth ride.
Feedback Linearization is used to cancel the effects, due to physics, mathematically with a robust term. After cancellation it behaves like a simple linear system. A PD controller then corrects the errors (with Kd and Kp)
Control inputs are rear wheel forces
Robust controller because the full vehicle dynamics are accessible but there are still uncertainties

planner.mlx
While the controller tracks the trajectory, the planner steps in only if safety function (control barrier function) is about to be violated. If the safety function remains positive, the robot never enters the obstacle and thus no action is taken. 
The car wants to follow a path using a normal controller
The CBF planner watches how close the car is to an obstacle
If the car is far away, it does nothing
If the car gets too close, it slightly adjusts the wheel forces to avoid a collision
The planner solves an optimization problem that minimally changes the wheel forces while remaining as close as possible to the regular controller output
MATLAB ‘Optimization’ Toolbox ~ quadprog() function

