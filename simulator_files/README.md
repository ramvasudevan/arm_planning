# Simulator Files for Arm Planning



## 1. Dependencies

To use the files in this repository, you will need the following MATLAB repositories on your path:

- [simulator](https://github.com/skousik/simulator)
- [RTD](https://github.com/ramvasudevan/RTD)

It might also be worth going through the [RTD Tutorial](https://github.com/skousik/RTD_tutorial) to get a feel for how the objects herein interact



## 2. Simulator Overview

This folder, when complete, should contain examples of `agent`, `planner`, and `world` files to be used with the `simulator` framework.

### 2.1 Simulator

The simulator takes in an agent, world, and planner, roughly, as follows:

```matlab
A = agent()
W = world() ; % or W = {world_1(), world_2(), ... , world_N()}
P = planner() ; % or P = {planner_1(), planner_2(), ... , planner_M()}

S = simulator(A,W,P)
```

Then, to run a simulation, one can just do:

```matlab
S.run()
```

This will use the planner to generate plans for the agent to navigate around in the world. If multiple worlds and/or planners are entered in the simulator, then, for each world, this will run the agent once for each planner (i.e., in a double-nested for loop).

In MATLAB pseudocode, the simulations roughly run as follows:

```matlab
while t < t_max and iter < iter_max
	% get information about the agent's current state
	agent_info = A.get_agent_info() ;
	
	% get info about the world relative to the agent
	world_info = W.get_world_info(agent_info) ;
	
	% create a new trajectory plan within P.t_plan and hand that to the agent
	% to execute; otherwise, the agent begins stopping
	try
    [time_des, input_des, traj_des] = P.replan(agent_info,world_info) ;
    A.move(P.t_move, time_des, input_des, traj_des) ;
  catch
		A.stop(P.t_move)
	end	
end
```

Note that both `A.move` and `A.stop` are called for the duration `P.t_move`. If `P.t_move` is equal to `P.t_plan` (the planning time limit), then the simulation enforces real-time planning. Since the simulator framework does not move the agent until a plan is created, one can allow the planner to operate slower than real time, which is useful for demonstration purposes.

### 2.2 Agent

An agent is a representation of a robot, with the following important properties:

- `A.state` is an `A.n_states`-by-`N` array where each column is the agent's state at the time given in `A.time`, which is a 1-by-`N` monotonically increasing vector.
- `A.input` is an `A.n_inputs`-by-`M` array where each column is the agent's input to be applied at the times given in `A.input_time`, which is a 1-by-`M` monotonically increasing vector. Typically, the input time and agent time are the same.

There are other useful properties, such as indices for different types of states (e.g, position or orientation), which one can find inside any agent file.



The agent also has the following important methods:

- `A.reset` sets the state, input, and time array back to some nominal starting value, typically arrays of zeros of the appropriate sizes.
- `A.get_agent_info` produces an `agent_info` structure that, at the very least, has the fields `state` and `time`. This structure is handed to the world and planner at each planning iteration in any simulation, so that the world and planner can react to the agent's motion.
- `A.move` takes in an amount of time, as well as a reference time, reference input, and reference trajectory (optional); the agent then attempts to execute this trajectory. The reference time, input, and trajectory should have the same number of rows as the agent's time, number of inputs, and number of states, respectively. The reference can have as many columns as are needed to specify the reference trajectory (it is up to the agent to decide whether to, e.g., use a zero or first-order hold between these columns to interpolate).
- `A.stop` takes in an amount of time, and executes the agent's default stopping behavior for that duration.
- `A.dynamics` takes in the current time, current state, and the reference time/input/trajectory, and outputs an `A.n_states`-by-1 vector of the agent's dynamics. This is the value of the derivative from an ordinary differential equation that represents the agent's motion, and is used to update `A.state` via numerical integration (by default, using `ode45` in the `A.integrator` method).

Note that the agent has a property, `A.LLC`, which is optional. This should be an instance of the simulator's `low_level_controller` class, and has the method `LLC.get_control_inputs` which can be called by the agent's dynamics to produce the control inputs to be used by the agent at each time iteration.



### 2.3 Planner

Shreyas please write this



### 2.4 World

Shreyas also write this



## 3. How To Use the Robot Arm Agents

### 3.1 2-D Arm Example

Let's start with a 2-D arm agent:

```matlab
A = robot_arm_2D_2DOF() ;
```

This is a 2-link, 2-DOF, 2-D arm. It has 4 states, which are the position and speed of its two rotational joints. You can see what it looks like with the following:

```matlab
figure(1) ;
plot(A) ;
axis equal ; grid on ;
```

Let's put it at a random state and see what it looks like:

```matlab
A.state = rand(A.n_states,1) ;
plot(A) 
```

We can also give it feedforward inputs to execute, and have it move:

```matlab
t_move = 3 ; % number of seconds to move
T = [0 2 4] ; % time vector
U = 2*rand(A.n_inputs, length(T)) - 1 ; % feedforward input

% call the move method, which runs numerical integration in the background
A.move(t_move, T, U)

% animate the agent:
A.animate() ; % you can also call this as animate(A)
```

You should see the arm flail around.



### 3.2 Low-level Controller

This arm, by default, has a `robot_arm_PID_LLC` low-level controller, which is the `A.LLC` property. You could write a different low-level controller if you wanted, but it's probably easier to just tune the gains of the existing LLC, which are:

```matlab
A.LLC.K_ff % feedforward gain
A.LLC.K_p % proportional gain
A.LLC.K_i % integration error gain
A.LLC.K_d % derivative gain
```

These are gain arrays that are automagically sized appropriately for the agent's number of states and inputs. The easiest way to adjust them would be to multiple the existing arrays by some value. For example:

```matlab
A.LLC.K_p = 10.*A.LLC.K_p ;
```

This way you don't have to resize things manually.



### 3.3 Tracking a Reference Point

Now that we know the LLC exists, let's get the arm to a setpoint. The following code is all in `arm_example_1_tracking_reference_point.m`.

To start, let's get a fresh copy of the arm:

```matlab
clear ; clf ; clc ;
A = robot_arm_2D_2DOF() ;
```

Now, we'll try to make the arm's first joint get to `pi/2` radians while the second joint stays at 0 radians. First, let's make the reference time, input, and trajectory:

```matlab
% reference time and input:
T = [0 4] ; % we'll give it 4 seconds for the maneuver
U = zeros(A.n_inputs, size(T,2)) ; % no feedforward input this time

% reference joint position:
joint_1 = [pi/2, pi/2] ;
joint_2 = [0, 0] ;

% reference joint speeds, which try to make the arm stop at the end configuration
joint_spd_1 = [(pi/2)/T(2), 0] ;
joint_spd_2 = [0, 0] ;

% concatenate joint and speed references in to a reference trajectory:
Z = [joint_1 ;
     joint_spd_1 ;
     joint_2 ;
     joint_spd_2 ] ;
     
% now, move the agent:
A.move(T(2), T, U, Z)
```

Now let's see what happened:

```matlab
animate(A)
```

You can see how the first joint overshoots and then corrects to get to the target, while the second joint stays at 0 degree the whole time. Currently, the arm is not modeling any sort of inertia or gravity, so the second joint does not have to apply any correcting inputs — but we'll be adding that eventually.



### 3.4 Tracking a Reference Trajectory

Now let's try tracking a reference trajectory. This code is all in `arm_example_2_tracking_reference_trajectory.m`. The point of this example is to try playing with the arm's control gains.

First, let's reset the arm:

```matlab
A.reset()
```

Note that the `reset` method deletes the arm's previously executed time, input, and state trajectory, and sets them all to zero.

Now, let's pick some control gains:

```matlab
k_ff = 0 ; % no feedforward gain
k_p = 100 ;
k_i = 0.01 ;
k_d = 10 ; 
```

We can plug these gains into the arm's default PID controller with the following method:

```
A.LLC.set_gains(k_ff,k_p,k_i,k_d)
```

You could also call this method as follows:

```matlab
A.LLC.set_gains('P',k_p,'I',k_i,'D',k_d,'FF',k_ff)
```

Note that the keyword order does not matter.



Now, let's make a sinusoidal reference trajectory for both joints:

```matlab
t_move = 10 ;
t_total = 10 ;
T = linspace(0,t_total) ;
U = zeros(A.n_inputs, size(T,2)) ;
Z = [1 - cos(T) ;
     sin(T) ;
     sin(T) ;
     cos(T)] ;
```

Move the arm as usual:

```matlab
A.move(t_move,T,U,Z)
```

And, animate!

```matlab
figure(1) ; clf ;
animate(A)
```

With the gains we've chosen, two interesting things happen. First, due to the initial speed difference between the arm and its reference, the arm overshoots its speed command. Second, about 6 seconds into the trajectory, the arm bottoms out its first joint, which shows up as a flat spot in the position trajectory and a small bump in the speed trajectory. You can see that with the following lines:

```matlab
figure(2) ; clf ;

% get states and reference matched in time
Z_j = A.state(A.joint_state_indices,:) ;
Z_d = A.state(A.joint_speed_indices,:) ;
Z_ref_j = match_trajectories(A.time, T, Z(A.joint_state_indices,:), A.LLC.interp_method) ;
Z_ref_d = match_trajectories(A.time, T, Z(A.joint_speed_indices,:), A.LLC.interp_method) ;

% position
subplot(2,1,1) ;
plot(A.time',Z_ref_j','b:',A.time',Z_j','b')
title('position')
xlabel('rad')

% speed
subplot(2,1,2) ;
plot(A.time',Z_ref_d','b:',A.time',Z_d','b')
title('speed')
xlabel('time')
ylabel('rad/s')
```

Note that the gains we set earlier are, in fact, the default gains of the controller. However, the following gains work much better for tracking this trajectory:

```matlab
k_p = 20 ;
k_d = 100 ; 
k_i = 0.01 ;
```



### 3.5 Collision Checking

Besides tracking points and trajectories, we also need to be able to tell if the arm is hitting stuff. Let's use the agent's `get_agent_info` method to do this. This code is in `arm_example_3_collision_checking.m`.

First, let's make an arm and set it to a random joint configuration `q`:

```matlab
A = robot_arm_2D_2DOF() ;
q = rand(2,1) ; % because our arm has 2 DOFs
A.state(A.joint_state_indices) = q ;
```

Now, let's make a random obstacle with five sides that approximately fits in a 0.3 m square and is offset to the point (0.5,0.5).

```matlab
O = 0.3.*make_random_polygon(5) + [0.5;0.5] ;
```

To check for collisions between the arm and this polygon, first, we'll get the agent's info structure. This is because, when running things in the simulator framework, the world and planner (which often have to do collision checking) do not have direct access to the agent.

```matlab
I = A.get_agent_info()
```

We want to check if the agent, at the configuration `q`, is in collision with the obstacle `O`. First, let's get an approximation of the volume that the agent occupies. Conveniently, the `robot_arm_agent` hands us a function that returns this volume:

```matlab
V = I.get_collision_check_volume(q) ;
```

Now we can collision check with MATLAB's `polyxpoly` function:

```matlab
[x_int,y_int] = polyxpoly(V(1,:),V(2,:),O(1,:),O(2,:)) ;
```

If the outputs of this function are empty vectors, then we are collision free.

We can plot all this as follows:

```matlab
figure(1) ; clf ; hold on ; axis equal

plot(A)
plot(V(1,:),V(2,:),'r--','LineWidth',1.5)
patch('Vertices',O','Faces',[1:size(O,2), 1],...
    'FaceColor',[1 0 0],'FaceAlpha',0.3,...
    'EdgeColor',[0.5 0 0],'LineWidth',1.5)

if ~isempty(x_int)
    plot(x_int,y_int,'kx')
end
```



That wraps up this README tutorial for now. Go play!