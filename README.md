# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Model


states - [x,y,ψ,v]

Actuators: [δ,a]			

Equations:

            x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
			
			y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
			
			psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
			
			v_[t+1] = v[t] + a[t] * dt
			
			cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
			
			epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
			

Lf - measures the distance between the front of the vehicle and its center of gravity.
			
			
## Timestep Length and Elapsed Duration (N & dt)

N was chosen to be 20
dt was chosen to be 0.05

T = 1 second

Tried various values from dt raing till 0.1 and N changing to 40, but car wobbles and goes out of track.


## Polynomial Fitting and MPC Preprocessing

X and Y values were fitted using 3rd degree polynomial

 polyfit(xvals, yvals, 3);
 
 
## Model Predictive Control with Latency

Latency is incorporated into the vehicle model given above.

 
 


			

