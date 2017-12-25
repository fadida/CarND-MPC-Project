# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

This project works with Udacity self driving simulator (for term 2) and
uses a Model Predictive Controller in order to steer a vehicle around a track.

## MPC
The MPC is a controller that set its outputs based on a physical model that is integrated inside the controller.
This controller uses finite future horizon in order to take into acocunt the vehicle future state by solving an optimization problem.

### Future Horizon
In this project I choose to use future horizon of 2.5 seconds which is made by 10 slots of 0.25 seconds each.

I choose to use 10 timeslots becuase this number provides good balance for the controller, more then 10 makes
the computation slower and less timeslots hurts accuracy.

I choose to set the timeslot duration to 0.25 secs in order to make the responsive enough to handle the vehicle at
high speeds but also not too responsive becuase this project indroduce delay of 100ms which limits the reposivness of 
the controller.


### MPC input parameters
The MPC in this project takes in a state vector, actuators vector and track polynimial coefficients.

The state vector is: `[x, y, psi, v, cte, epsi]` where:
* `x` is the current x position.
* `y` is the current y position.
* `psi` is the current vehicle orientation.
* `v` is the current vehicle velocity.
* `cte` is the current cross track error.
* `epsi` is the current track orientation error.

The actuator values vector is: `[delta, a]` where:
* `delta` is the current steering angle of the vehicle.
* `a` is the current throttle of the vehicle.

The state vector and the actuator vectors were selected in order to match the kinematic model that will be exlained later.

After the MPC takes in the vector it start to setup the optimization problem by initalizing the future horizon to zero,
setting the variables bounds and creating the cost function and constraints.

### Track polynomial
The MPC need to take in the track polynoimal coefficients in order to calculate the track errors.
The CarND simulator provides track waypoints in map coordinates.

In order to simplify the MPC implementation I transformed the map waypoints to vehicle coordinates.
The transformation is done by setting the origin to the car position and by rotating the coordinates based on the car orientation.

After the waypoints were transformed I used `polyfit()` to create a polynomial with rank of 3 in order to represent the waypoints
as a continous line.
The polynomial is of rank 3 because most curves match a 3rd order polynomial.

### Variable bounds
The variable bounds for future states and actuator values are initialized to the minimum & maximum allowed values for each variable,
and the current state & actuator values is initialized to the value of the current state and actuator values.

### Cost function
The cost function purpose is to minimize several elements in order to make the car stay in track:
1. **The track errors** - The cost function tries to minimize the track and orientation errors in order for
                     the vehicle to stay in the middle of the track.

2. **The vehicle velocity** - The cost function regulates the vehicle velocity to make sure that it will stay somewhat
                         constant. Otherwise, if the velocity will continue to increase, the vehicle will become less
                         staible and will get out of track on curves.

3. **The vehicle actuator values** - The cost function tries to keep the actuator values to minimum.
                                Very large actuator values will cause very large veriation in velocity and motion
                                which can in turn make the vehicle motion less stable and cause the overcompensation
                                of future actuator values until the values reach saturation and the motion cannot be fixed
                                by the controller.

4. **Actuator values continuity** - Actuator values should be continous and not include large jumps in values.
                               The reason for this is that large jumps in actuator values can cause violent responces
                               from vehicle and the purpose of this contorller is also to keep motion as smooth as possible.

Those different element has different importence for the cost values becuase some of those can cause opposing actuator values
The importence is signified using the factors before each element is added to the cost function.

The factor I choose for the cost functions are:
* Track errors factor: 2000
* Velocity factor: 3
* Steering actuator factor: 40
* Throttle actuator factor: 20
* Continous steering factor: 1000
* Continous throttle factor: 100

We can see that my cost function is priortizing the need to correct the track errors and steer the vehicle thowards the center of the track, after that the cost is trying to keep the steering continous and after that meeting the other conditions.

I choose those factors in order to keep the vehicle on track with smooth steering behaviour.

### Cost constraints
The cost function is not sufficient to optimize the problem because the vehicle state is constrained by the the kinematic model.
For that reason, for each state in the future horizon the MPC ties it to the previous state by using the kinematic model.

### Kinematic Model
The model used in this project is the vehicle kinematic motion model.
The model goes as follows:

       x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
       y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
       psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
       v[t+1] = v[t] + a[t] * dt
       cte[t+1] = track_y[t] - y[t] + v[t] * sin(epsi[t]) * dt
       epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
       
Where:
* `x[t]` and `y[t]` are the vehicle position at time t.
* `psi[t]` is the vehicle orientation at time t. 
* `v[t]` is the vehicle velocity at time t.
* `a[t]` is the throttle of the vehicle at time t.
* `track_y[t]` is the target y position on time t.
* `epsi[t]` is the orientation error at time t.
* `dt` is the time delta between t to (t+1).
* `cte[t]` is the cross track error at time t.
* `psides[t]` is the orientation the vehicle should have at time t.
* `Lf` is a constant which is dependet on vehicle structure. 

In this project the MPC uses this model to constraint the state and actuator values when doing the optimization.

### Sending back
After the problem is defined and solved, the MPC sends the actuator values for t+1 back to the simulator.

## Latency handling 
In this project we add an artifical latency to simulator of 100ms in order to emulate response latency.
The way I handle the latency in this project is by passing the effective state to the MPC.

In order to avoid adding the letency into the cost function of the MPC, I calculate the state the vehicle will be in at time (t+latency).
The result of this method is that the MPC works on the state that will actualy change effectivly it works with no latency.


