function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% =================== Your code goes here ===================

% error coefficients & constants
kp_phi = 500;
kp_theta = 500;
kp_psi = 500; 

kd_phi = 250; 
kd_theta = 250;
kd_psi = 250;

kp1 = 10;
kp2 = 19;
kp3 = 1000;

kd1 = 97;
kd2 = 97;
kd3 = 97;

m = params.mass;
g = params.gravity;
I = params.I;
L = params.arm_length;

% position and vels
ep_x = des_state.pos(1) - state.pos(1);
ev_x = des_state.vel(1) - state.vel(1);
ep_y = des_state.pos(2) - state.pos(2);   
ev_y = des_state.vel(2) - state.vel(2);
ep_z = des_state.pos(3) - state.pos(3);
ev_z = des_state.vel(3) - state.vel(3);

% des acceclaration
x_ddot = des_state.acc(1);
y_ddot = des_state.acc(2);
z_ddot = des_state.acc(3);

% angles
phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);

rdes =  des_state.yawdot;

%angular acceleration
p = state.omega(1); 
q = state.omega(2); 
r = state.omega(3);


% =================== Your code ends here ===================
%desired accelerations
rddot1_des = x_ddot +((kd1 * ev_x) + (kp1 * ep_x));
rddot2_des = y_ddot +((kd2 * ev_y) + (kp2 * ep_y));
rddot3_des = z_ddot +((kd3 * ev_z) + (kp3 * ep_z));

%desired angles
psi_des = des_state.yaw;
phi_des = 1/g * ( (rddot1_des * sin(psi_des)) - (rddot2_des * cos(psi_des)) );
theta_des =  1/g * ((rddot1_des * cos(psi_des)) + (rddot2_des * sin(psi_des)) );

u1 = m*g + m *(rddot3_des);
F = u1;
pdes = 0;
qdes = 0;

%matrix elem for u2
first = (kp_phi *(phi_des - phi)) + (kd_phi * (pdes - p));
second = (kp_theta *(theta_des - theta)) + (kd_theta *(qdes - q));
third = (kp_psi * (psi_des - psi)) + (kd_psi * (rdes - r));

u2 = [first;second;third];
M = I * u2;
end
