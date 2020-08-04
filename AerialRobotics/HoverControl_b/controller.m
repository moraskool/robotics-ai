function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
        
%u1 = 0;
%u2 = 0;
phi_c_dot = 0;
phi_c_ddot = 0;

% FILL IN YOUR CODE HERE
% error coefficients
kv_z = 120; 
kp_y = 19;
kv_y = 80;
kp_z =  10;
kv_phi = 250; 
kp_phi = 500;

% constants
m = params.mass;
g = params.gravity;
Ixx = params.Ixx;
L = params.arm_length;

% position and vels
ep_y = des_state.pos(1) - state.pos(1);   
ev_y = des_state.vel(1) - state.vel(1);
ep_z = des_state.pos(2) - state.pos(2);
ev_z = des_state.vel(2) - state.vel(2);

z_ddot = des_state.acc(2);
y_ddot = des_state.acc(1);

% angles
phi = state.rot;
phi_dot = state.omega;
phi_c = (- 1/g) * (y_ddot + (kv_y * ev_y) + (kp_y * ep_y));

% cal
u1 = m * (g + z_ddot + (kv_z * ev_z) + (kp_z *ep_z));
u2 = Ixx * (phi_c_ddot + kv_phi*(phi_c_dot - phi_dot) + kp_phi*(phi_c - phi));
end

